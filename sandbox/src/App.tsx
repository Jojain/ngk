import { useEffect, useRef, useState, type PointerEvent } from "react";
import Editor, { type BeforeMount } from "@monaco-editor/react";
import { SceneShell, VizSceneView, type VizScene } from "@ngk/viewer";
import { boltExample } from "./bolt";
import { ngkModuleTypes, scriptTypes } from "./editorTypes";
import type { WorkerRequest, WorkerResponse } from "./protocol";
import { codeFromUrl, shareCode } from "./urlCodec";

const STORAGE_KEY = "ngk-sandbox-code-v1";

type OutputLine = { level: "log" | "warn" | "error"; text: string };

function initialCode() {
  try {
    return localStorage.getItem(STORAGE_KEY) ?? boltExample;
  } catch {
    return boltExample;
  }
}

const setupEditor: BeforeMount = (monaco) => {
  monaco.languages.typescript.typescriptDefaults.addExtraLib(ngkModuleTypes, "file:///node_modules/ngk-wasm/index.d.ts");
  monaco.languages.typescript.typescriptDefaults.addExtraLib(scriptTypes, "file:///ngk-sandbox/globals.d.ts");
  monaco.languages.typescript.typescriptDefaults.setCompilerOptions({
    target: monaco.languages.typescript.ScriptTarget.ES2022,
    noEmit: true,
    strict: true,
  });
};

export default function App() {
  const [code, setCode] = useState(initialCode);
  const [scene, setScene] = useState<VizScene | null>(null);
  const [lines, setLines] = useState<OutputLine[]>([]);
  const [running, setRunning] = useState(false);
  const [status, setStatus] = useState("Ready");
  const [editorWidth, setEditorWidth] = useState(45);
  const [consoleHeight, setConsoleHeight] = useState(190);
  const workerRef = useRef<Worker | null>(null);
  const workspaceRef = useRef<HTMLDivElement | null>(null);
  const rightRef = useRef<HTMLDivElement | null>(null);
  const dragRef = useRef<{ kind: "columns" | "rows"; start: number; value: number } | null>(null);

  useEffect(() => {
    try { localStorage.setItem(STORAGE_KEY, code); } catch { /* storage may be disabled */ }
  }, [code]);

  useEffect(() => {
    let active = true;
    void codeFromUrl().then((sharedCode) => {
      if (active && sharedCode) setCode(sharedCode);
    });
    return () => { active = false; };
  }, []);

  useEffect(() => () => workerRef.current?.terminate(), []);

  function stop() {
    workerRef.current?.terminate();
    workerRef.current = null;
    setRunning(false);
    setStatus("Cancelled");
    setLines((previous) => [...previous, { level: "warn", text: "Run cancelled." }]);
  }

  function run() {
    workerRef.current?.terminate();
    const worker = new Worker(new URL("./runner.worker.ts", import.meta.url), { type: "module" });
    workerRef.current = worker;
    setScene(null);
    setLines([]);
    setRunning(true);
    setStatus("Starting…");
    worker.onmessage = (event: MessageEvent<WorkerResponse>) => {
      if (workerRef.current !== worker) return;
      const message = event.data;
      if (message.kind === "ready") setStatus("Running…");
      if (message.kind === "log") setLines((previous) => [...previous, message]);
      if (message.kind === "scene") setScene(message.scene);
      if (message.kind === "done") {
        if (!message.shown) setLines((previous) => [...previous, { level: "warn", text: "Script finished without show(shape)." }]);
        setStatus("Complete");
        setRunning(false);
        worker.terminate();
        workerRef.current = null;
      }
      if (message.kind === "error") {
        setLines((previous) => [...previous, { level: "error", text: message.text }]);
        setStatus("Error");
        setRunning(false);
        worker.terminate();
        workerRef.current = null;
      }
    };
    worker.onerror = (event) => {
      if (workerRef.current !== worker) return;
      setLines((previous) => [...previous, { level: "error", text: event.message }]);
      setStatus("Error");
      setRunning(false);
      worker.terminate();
      workerRef.current = null;
    };
    const request: WorkerRequest = { kind: "run", code };
    worker.postMessage(request);
  }

  async function share() {
    try {
      const result = await shareCode(code);
      setLines((previous) => [...previous, {
        level: result === "copied" ? "log" : "warn",
        text: result === "copied"
          ? "Share URL copied to clipboard."
          : result === "too-long"
            ? "This script is too long for a share URL."
            : "Clipboard access is unavailable; copy the URL from the address bar.",
      }]);
    } catch (error) {
      setLines((previous) => [...previous, { level: "error", text: error instanceof Error ? error.message : String(error) }]);
    }
  }

  function beginDrag(kind: "columns" | "rows", event: PointerEvent<HTMLDivElement>) {
    event.currentTarget.setPointerCapture(event.pointerId);
    dragRef.current = { kind, start: kind === "columns" ? event.clientX : event.clientY, value: kind === "columns" ? editorWidth : consoleHeight };
  }

  function moveDrag(event: PointerEvent<HTMLDivElement>) {
    const drag = dragRef.current;
    if (!drag) return;
    if (drag.kind === "columns") {
      const width = workspaceRef.current?.clientWidth ?? 1;
      setEditorWidth(Math.min(75, Math.max(25, drag.value + 100 * (event.clientX - drag.start) / width)));
    } else {
      const height = rightRef.current?.clientHeight ?? 1;
      setConsoleHeight(Math.min(height - 130, Math.max(110, drag.value - (event.clientY - drag.start))));
    }
  }

  return (
    <div className="app">
      <header className="toolbar">
        <div className="brand"><strong>NGK</strong><span>Sandbox</span></div>
        <div className="toolbar-actions">
          <span className="status">{status}</span>
          <button onClick={() => setCode(boltExample)} title="Load the bolt example">Bolt example</button>
          <button onClick={() => void share()} title="Copy a compressed URL for this script">Share</button>
          {running ? <button className="cancel" onClick={stop}>Cancel</button> : <button className="run" onClick={run}>Run</button>}
        </div>
      </header>
      <div ref={workspaceRef} className="workspace" style={{ gridTemplateColumns: `${editorWidth}% 6px minmax(0, 1fr)` }}>
        <section className="editor-pane" aria-label="Code editor">
          <div className="pane-title">model.ts <span>TypeScript / JavaScript</span></div>
          <Editor
            path="model.ts"
            language="typescript"
            theme="vs-dark"
            value={code}
            onChange={(value) => setCode(value ?? "")}
            beforeMount={setupEditor}
            options={{ fontSize: 13, minimap: { enabled: false }, automaticLayout: true, scrollBeyondLastLine: false, tabSize: 2 }}
          />
        </section>
        <div className="resize-handle columns" role="separator" aria-orientation="vertical" onPointerDown={(event) => beginDrag("columns", event)} onPointerMove={moveDrag} onPointerUp={() => { dragRef.current = null; }} />
        <div ref={rightRef} className="right-pane" style={{ gridTemplateRows: `minmax(0, 1fr) 6px ${consoleHeight}px` }}>
          <section className="viewer-pane" aria-label="3D viewer">
            <div className="pane-title">Viewer</div>
            <SceneShell scene={scene}>
              {scene && <VizSceneView scene={scene} showDarts={false} visibleAlphas={new Set()} showVertices={false} showWorldFrame={false} />}
            </SceneShell>
            {!scene && <div className="viewer-empty">Run a script that calls <code>show(shape)</code></div>}
          </section>
          <div className="resize-handle rows" role="separator" aria-orientation="horizontal" onPointerDown={(event) => beginDrag("rows", event)} onPointerMove={moveDrag} onPointerUp={() => { dragRef.current = null; }} />
          <section className="console-pane" aria-label="Console">
            <div className="pane-title">Console <button onClick={() => setLines([])}>Clear</button></div>
            <div className="console-output" role="log">
              {lines.map((line, index) => <pre key={index} className={line.level}>{line.text}</pre>)}
            </div>
          </section>
        </div>
      </div>
    </div>
  );
}
