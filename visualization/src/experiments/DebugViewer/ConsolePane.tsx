import {
  useEffect,
  useMemo,
  useRef,
  useState,
  type KeyboardEvent,
  type PointerEvent as ReactPointerEvent,
} from "react";

import type { Kernel } from "../../kernel/useKernel";
import {
  debugTopologyKind,
  sameTopology,
  type DebugTopologyEntity,
  type HydratedDebugDump,
} from "../../kernel/debugViewer";
import { objectPreview } from "./objectPreviews";

type ConsoleBindings = {
  shape: HydratedDebugDump["shape"];
  shapes: HydratedDebugDump["shapes"];
  gmap: HydratedDebugDump["gmap"];
  gmaps: HydratedDebugDump["gmaps"];
  ngk: Kernel;
};

type ConsoleEntry = {
  id: number;
  source: string;
  value: unknown;
  error: boolean;
};

type Completion = {
  label: string;
  start: number;
  end: number;
};

const DEFAULT_CONSOLE_HEIGHT = 360;
const MIN_CONSOLE_HEIGHT = 180;
const CONSOLE_VIEWPORT_MARGIN = 20;
const MAX_INSPECTOR_DEPTH = 6;
const INSPECTABLE_ACCESSORS = new Set([
  "darts",
  "edges",
  "faces",
  "innerLoops",
  "innerShells",
  "loops",
  "profiles",
  "sheets",
  "shells",
  "solids",
  "vertices",
]);

export function ConsolePane({
  dump,
  kernel,
  highlightedTopology,
  onToggleTopologyHighlight,
}: {
  dump: HydratedDebugDump | null;
  kernel: Kernel | null;
  highlightedTopology: DebugTopologyEntity | null;
  onToggleTopologyHighlight: (entity: DebugTopologyEntity) => void;
}) {
  const [source, setSource] = useState("shape");
  const [caret, setCaret] = useState(5);
  const [entries, setEntries] = useState<ConsoleEntry[]>([]);
  const [history, setHistory] = useState<string[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [completionIndex, setCompletionIndex] = useState(0);
  const [collapsed, setCollapsed] = useState(false);
  const [consoleHeight, setConsoleHeight] = useState(defaultConsoleHeight);
  const inputRef = useRef<HTMLTextAreaElement | null>(null);
  const outputRef = useRef<HTMLDivElement | null>(null);
  const resizeRef = useRef<{
    pointerId: number;
    startY: number;
    startHeight: number;
  } | null>(null);

  const bindings = useMemo<ConsoleBindings | null>(
    () =>
      dump && kernel
        ? {
            shape: dump.shape,
            shapes: dump.shapes,
            gmap: dump.gmap,
            gmaps: dump.gmaps,
            ngk: kernel,
          }
        : null,
    [dump, kernel],
  );
  const completions = useMemo(
    () => (bindings ? complete(source, caret, bindings) : []),
    [bindings, caret, source],
  );

  useEffect(() => {
    setCompletionIndex(0);
  }, [source, caret]);

  useEffect(() => {
    if (!bindings) return;
    const globals = window as unknown as Record<string, unknown>;
    const previous = new Map<string, unknown>();
    for (const [name, value] of Object.entries(bindings)) {
      previous.set(name, globals[name]);
      globals[name] = value;
    }
    return () => {
      for (const [name, value] of Object.entries(bindings)) {
        if (globals[name] !== value) continue;
        const old = previous.get(name);
        if (old === undefined) delete globals[name];
        else globals[name] = old;
      }
    };
  }, [bindings]);

  useEffect(() => {
    outputRef.current?.scrollTo({ top: outputRef.current.scrollHeight });
  }, [entries]);

  useEffect(() => {
    const fitToViewport = () => {
      setConsoleHeight((height) => clampConsoleHeight(height));
    };
    window.addEventListener("resize", fitToViewport);
    return () => window.removeEventListener("resize", fitToViewport);
  }, []);

  const execute = async () => {
    const expression = source.trim();
    if (!bindings || expression.length === 0) return;
    let value: unknown;
    let error = false;
    try {
      value = evaluateForConsole(expression, bindings);
      if (value instanceof Promise) value = await value;
    } catch (cause) {
      value = cause;
      error = true;
    }
    setEntries((current) => [
      ...current,
      { id: current.at(-1)?.id ? current.at(-1)!.id + 1 : 1, source: expression, value, error },
    ]);
    setHistory((current) => [expression, ...current.filter((item) => item !== expression)]);
    setHistoryIndex(-1);
  };

  const acceptCompletion = (completion = completions[completionIndex]) => {
    if (!completion) return;
    const next =
      source.slice(0, completion.start) +
      completion.label +
      source.slice(completion.end);
    const nextCaret = completion.start + completion.label.length;
    setSource(next);
    setCaret(nextCaret);
    requestAnimationFrame(() => {
      inputRef.current?.focus();
      inputRef.current?.setSelectionRange(nextCaret, nextCaret);
    });
  };

  const onKeyDown = (event: KeyboardEvent<HTMLTextAreaElement>) => {
    if (event.key === "Tab" && completions.length > 0) {
      event.preventDefault();
      acceptCompletion();
      return;
    }
    if (event.key === "ArrowDown" && completions.length > 0) {
      event.preventDefault();
      setCompletionIndex((index) => (index + 1) % completions.length);
      return;
    }
    if (event.key === "ArrowUp" && completions.length > 0) {
      event.preventDefault();
      setCompletionIndex(
        (index) => (index - 1 + completions.length) % completions.length,
      );
      return;
    }
    if (event.key === "ArrowUp" && history.length > 0) {
      event.preventDefault();
      const next = Math.min(historyIndex + 1, history.length - 1);
      setHistoryIndex(next);
      setSource(history[next]);
      setCaret(history[next].length);
      return;
    }
    if (event.key === "ArrowDown" && historyIndex >= 0) {
      event.preventDefault();
      const next = historyIndex - 1;
      setHistoryIndex(next);
      setSource(next >= 0 ? history[next] : "");
      setCaret(next >= 0 ? history[next].length : 0);
      return;
    }
    if (event.key === "Enter" && !event.shiftKey) {
      event.preventDefault();
      void execute();
    }
  };

  const resizeFromPointer = (event: ReactPointerEvent<HTMLDivElement>) => {
    const resize = resizeRef.current;
    if (!resize || resize.pointerId !== event.pointerId) return;
    setConsoleHeight(
      clampConsoleHeight(resize.startHeight + resize.startY - event.clientY),
    );
  };

  const finishResize = (event: ReactPointerEvent<HTMLDivElement>) => {
    if (resizeRef.current?.pointerId !== event.pointerId) return;
    resizeRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
  };

  const resizeFromKeyboard = (event: KeyboardEvent<HTMLDivElement>) => {
    if (event.key !== "ArrowUp" && event.key !== "ArrowDown") return;
    event.preventDefault();
    const delta = event.key === "ArrowUp" ? 24 : -24;
    setConsoleHeight((height) => clampConsoleHeight(height + delta));
  };

  return (
    <section
      className={
        "debug-section debug-console-panel" + (collapsed ? " is-collapsed" : "")
      }
      style={collapsed ? undefined : { height: consoleHeight }}
    >
      {!collapsed && (
        <div
          className="debug-console-resize-handle"
          role="separator"
          aria-label="Resize shape console"
          aria-orientation="horizontal"
          aria-valuemin={minimumConsoleHeight()}
          aria-valuemax={maximumConsoleHeight()}
          aria-valuenow={consoleHeight}
          tabIndex={0}
          title="Drag to resize; double-click to reset"
          onDoubleClick={() =>
            setConsoleHeight(defaultConsoleHeight())
          }
          onKeyDown={resizeFromKeyboard}
          onPointerDown={(event) => {
            resizeRef.current = {
              pointerId: event.pointerId,
              startY: event.clientY,
              startHeight: consoleHeight,
            };
            event.currentTarget.setPointerCapture(event.pointerId);
            event.preventDefault();
          }}
          onPointerMove={resizeFromPointer}
          onPointerUp={finishResize}
          onPointerCancel={finishResize}
        />
      )}
      <div className="debug-panel-header">
        <h2>Shape console</h2>
        <div className="debug-header-actions">
          <span>{dump ? `${dump.shapes.length} shape${dump.shapes.length === 1 ? "" : "s"}` : "offline"}</span>
          <button type="button" onClick={() => setEntries([])} disabled={entries.length === 0}>
            Clear
          </button>
          <button
            type="button"
            aria-expanded={!collapsed}
            aria-controls="debug-shape-console-content"
            onClick={() => setCollapsed((value) => !value)}
          >
            {collapsed ? "Expand" : "Fold"}
          </button>
        </div>
      </div>
      <div
        className="debug-console-output"
        id="debug-shape-console-content"
        ref={outputRef}
      >
        {entries.length === 0 && (
          <div className="debug-console-help">
            Try <code>shape.faces</code> and unfold the result, or run{" "}
            <code>shape.edges()[0].start.point</code>. Tab completes members;
            Enter runs, Shift+Enter adds a line.
          </div>
        )}
        {entries.map((entry) => (
          <div className={`debug-console-entry${entry.error ? " error" : ""}`} key={entry.id}>
            <div className="debug-console-command"><span>›</span>{entry.source}</div>
            <InspectableValue
              value={entry.value}
              highlightedTopology={highlightedTopology}
              onToggleTopologyHighlight={onToggleTopologyHighlight}
            />
          </div>
        ))}
      </div>
      <div className="debug-console-input-wrap">
        <span className="debug-console-prompt">›</span>
        <textarea
          ref={inputRef}
          value={source}
          disabled={!bindings}
          rows={1}
          spellCheck={false}
          aria-label="JavaScript shape console"
          placeholder={bindings ? "Explore shape…" : "Waiting for a shape…"}
          onChange={(event) => {
            setSource(event.currentTarget.value);
            setCaret(event.currentTarget.selectionStart);
          }}
          onClick={(event) => setCaret(event.currentTarget.selectionStart)}
          onSelect={(event) => setCaret(event.currentTarget.selectionStart)}
          onKeyDown={onKeyDown}
        />
        <button type="button" onClick={() => void execute()} disabled={!bindings || !source.trim()}>
          Run
        </button>
        {completions.length > 0 && (
          <div className="debug-console-completions">
            {completions.slice(0, 12).map((completion, index) => (
              <button
                type="button"
                className={index === completionIndex ? "active" : ""}
                key={completion.label}
                onMouseDown={(event) => {
                  event.preventDefault();
                  acceptCompletion(completion);
                }}
              >
                {completion.label}
              </button>
            ))}
          </div>
        )}
      </div>
    </section>
  );
}

function maximumConsoleHeight(): number {
  return Math.max(120, window.innerHeight - CONSOLE_VIEWPORT_MARGIN);
}

function minimumConsoleHeight(): number {
  return Math.min(MIN_CONSOLE_HEIGHT, maximumConsoleHeight());
}

function defaultConsoleHeight(): number {
  return clampConsoleHeight(
    Math.min(DEFAULT_CONSOLE_HEIGHT, window.innerHeight * 0.46),
  );
}

function clampConsoleHeight(height: number): number {
  return Math.min(
    Math.max(height, minimumConsoleHeight()),
    maximumConsoleHeight(),
  );
}

function evaluate(source: string, bindings: ConsoleBindings): unknown {
  const names = Object.keys(bindings);
  const values = Object.values(bindings);
  const run = new Function(...names, `"use strict"; return (${source});`);
  return run(...values);
}

function evaluateForConsole(
  source: string,
  bindings: ConsoleBindings,
): unknown {
  const value = evaluate(source, bindings);
  if (typeof value !== "function") return value;

  const member = source.match(/^(.*)\.([A-Za-z_$][\w$]*)$/s);
  if (!member || !isInspectableAccessor(member[2], value)) return value;
  const owner = evaluate(member[1], bindings);
  return value.call(owner);
}

function complete(
  source: string,
  caret: number,
  bindings: ConsoleBindings,
): Completion[] {
  const before = source.slice(0, caret);
  const member = before.match(
    /([A-Za-z_$][\w$]*(?:(?:\.[A-Za-z_$][\w$]*)|(?:\([^()\n]*\))|(?:\[[^\]\n]*\]))*)\.([A-Za-z_$][\w$]*)?$/,
  );
  if (member) {
    try {
      const target = evaluate(member[1], bindings);
      const prefix = member[2] ?? "";
      return propertyNames(target)
        .filter(
          (name) =>
            name !== prefix && name.toLowerCase().startsWith(prefix.toLowerCase()),
        )
        .slice(0, 12)
        .map((label) => ({
          label,
          start: caret - prefix.length,
          end: caret,
        }));
    } catch {
      return [];
    }
  }

  const root = before.match(/([A-Za-z_$][\w$]*)$/);
  if (!root) return [];
  const prefix = root[1];
  return Object.keys(bindings)
    .filter(
      (name) => name !== prefix && name.toLowerCase().startsWith(prefix.toLowerCase()),
    )
    .slice(0, 12)
    .map((label) => ({ label, start: caret - prefix.length, end: caret }));
}

function propertyNames(value: unknown): string[] {
  if ((typeof value !== "object" || value === null) && typeof value !== "function") {
    return [];
  }
  const names = new Set<string>();
  let current: object | null = value as object;
  while (current && current !== Object.prototype) {
    for (const name of Object.getOwnPropertyNames(current)) {
      if (name !== "constructor" && name !== "free" && !name.startsWith("__")) {
        names.add(name);
      }
    }
    current = Object.getPrototypeOf(current) as object | null;
  }
  return [...names].sort((left, right) => left.localeCompare(right));
}

function InspectableValue({
  value,
  depth = 0,
  ancestors = new Set<object>(),
  defaultOpen = false,
  highlightedTopology,
  onToggleTopologyHighlight,
}: {
  value: unknown;
  depth?: number;
  ancestors?: Set<object>;
  defaultOpen?: boolean;
  highlightedTopology: DebugTopologyEntity | null;
  onToggleTopologyHighlight: (entity: DebugTopologyEntity) => void;
}) {
  const [open, setOpen] = useState(defaultOpen);
  const topology = debugTopologyKind(value)
    ? (value as DebugTopologyEntity)
    : null;
  const highlighted =
    topology && highlightedTopology
      ? sameTopology(topology, highlightedTopology)
      : false;
  if (!isExpandable(value) || depth >= MAX_INSPECTOR_DEPTH) {
    if (topology) {
      return (
        <TopologyReference
          value={topology}
          highlighted={highlighted}
          onToggle={onToggleTopologyHighlight}
        />
      );
    }
    return <span className="debug-console-value">{preview(value)}</span>;
  }
  if (ancestors.has(value)) {
    return <span className="debug-console-value">[Circular]</span>;
  }

  const nextAncestors = new Set(ancestors);
  nextAncestors.add(value);
  return (
    <div className="debug-console-object">
      <div className="debug-console-object-summary">
        <button
          type="button"
          className="debug-console-disclosure"
          aria-label={(open ? "Collapse " : "Expand ") + preview(value)}
          aria-expanded={open}
          onClick={() => setOpen(!open)}
        >
          <span>{open ? "▾" : "▸"}</span>
          {!topology && preview(value)}
        </button>
        {topology && (
          <TopologyReference
            value={topology}
            highlighted={highlighted}
            onToggle={onToggleTopologyHighlight}
          />
        )}
      </div>
      {open && (
        <div className="debug-console-properties">
          {properties(value).map((property) => (
            <InspectableProperty
              key={property.name}
              owner={value}
              property={property}
              depth={depth}
              ancestors={nextAncestors}
              highlightedTopology={highlightedTopology}
              onToggleTopologyHighlight={onToggleTopologyHighlight}
            />
          ))}
        </div>
      )}
    </div>
  );
}

function TopologyReference({
  value,
  highlighted,
  onToggle,
}: {
  value: DebugTopologyEntity;
  highlighted: boolean;
  onToggle: (entity: DebugTopologyEntity) => void;
}) {
  const kind = debugTopologyKind(value);
  return (
    <button
      type="button"
      className={
        "debug-console-topology-reference" + (highlighted ? " selected" : "")
      }
      aria-label={(highlighted ? "Unhighlight " : "Highlight ") + kind}
      aria-pressed={highlighted}
      title={(highlighted ? "Unhighlight " : "Highlight ") + kind + " " + value.key}
      onClick={() => onToggle(value)}
    >
      {preview(value)}
    </button>
  );
}

type InspectedProperty = {
  name: string;
  child?: unknown;
  method: boolean;
  accessor: boolean;
};

function InspectableProperty({
  owner,
  property,
  depth,
  ancestors,
  highlightedTopology,
  onToggleTopologyHighlight,
}: {
  owner: object;
  property: InspectedProperty;
  depth: number;
  ancestors: Set<object>;
  highlightedTopology: DebugTopologyEntity | null;
  onToggleTopologyHighlight: (entity: DebugTopologyEntity) => void;
}) {
  const [open, setOpen] = useState(false);
  const [result, setResult] = useState<{
    value: unknown;
    error: boolean;
  } | null>(null);

  if (!property.method) {
    return (
      <div className="debug-console-property">
        <span>{property.name}</span>
        <InspectableValue
          value={property.child}
          depth={depth + 1}
          ancestors={ancestors}
          highlightedTopology={highlightedTopology}
          onToggleTopologyHighlight={onToggleTopologyHighlight}
        />
      </div>
    );
  }

  if (!property.accessor) {
    return (
      <div className="debug-console-property">
        <span>{property.name}</span>
        <code>ƒ {property.name}()</code>
      </div>
    );
  }

  const toggle = () => {
    const nextOpen = !open;
    setOpen(nextOpen);
    if (!nextOpen || result) return;
    try {
      const accessor = property.child as (...args: never[]) => unknown;
      setResult({ value: accessor.call(owner), error: false });
    } catch (cause) {
      setResult({ value: cause, error: true });
    }
  };

  return (
    <div className="debug-console-property">
      <span>{property.name}</span>
      <div className="debug-console-accessor">
        <button
          type="button"
          className="debug-console-disclosure"
          aria-expanded={open}
          onClick={toggle}
        >
          <span>{open ? "▾" : "▸"}</span>
          <code>ƒ {property.name}()</code>
        </button>
        {open && result && (
          <div
            className={
              "debug-console-accessor-result" + (result.error ? " error" : "")
            }
          >
            <InspectableValue
              value={result.value}
              depth={depth + 1}
              ancestors={ancestors}
              defaultOpen
              highlightedTopology={highlightedTopology}
              onToggleTopologyHighlight={onToggleTopologyHighlight}
            />
          </div>
        )}
      </div>
    </div>
  );
}

function isExpandable(value: unknown): value is object {
  return typeof value === "object" && value !== null;
}

function preview(value: unknown): string {
  if (value === undefined) return "undefined";
  if (value === null) return "null";
  if (typeof value === "string") return JSON.stringify(value);
  if (typeof value === "number" || typeof value === "boolean" || typeof value === "bigint") {
    return String(value);
  }
  if (typeof value === "function") return `ƒ ${value.name || "anonymous"}()`;
  if (value instanceof Error) return `${value.name}: ${value.message}`;
  if (Array.isArray(value)) return arrayPreview(value);
  if (ArrayBuffer.isView(value)) {
    const length = "length" in value ? Number(value.length) : value.byteLength;
    return `${value.constructor.name}(${length})`;
  }
  return objectPreview(value) ?? value.constructor?.name ?? "Object";
}

function arrayPreview(value: unknown[]): string {
  if (value.length === 0) return "Array(0)";
  const names = new Set(
    value
      .slice(0, 20)
      .map((item) =>
        typeof item === "object" && item !== null
          ? item.constructor?.name
          : undefined,
      )
      .filter((name): name is string => Boolean(name) && name !== "Object"),
  );
  if (names.size !== 1) return "Array(" + value.length + ")";
  return [...names][0] + "[" + value.length + "]";
}

function isInspectableAccessor(name: string, value: unknown): boolean {
  return typeof value === "function" && INSPECTABLE_ACCESSORS.has(name);
}

function properties(value: object): InspectedProperty[] {
  if (Array.isArray(value) || ArrayBuffer.isView(value)) {
    return Array.from(value as ArrayLike<unknown>)
      .slice(0, 100)
      .map((child, index) => ({
        name: String(index),
        child,
        method: false,
        accessor: false,
      }));
  }

  return propertyNames(value)
    .slice(0, 100)
    .map((name) => {
      try {
        const child = (value as Record<string, unknown>)[name];
        return {
          name,
          child,
          method: typeof child === "function",
          accessor: isInspectableAccessor(name, child),
        };
      } catch (cause) {
        return { name, child: cause, method: false, accessor: false };
      }
    });
}
