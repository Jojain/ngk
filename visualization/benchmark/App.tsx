import { useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { colors, loadMesh, preview } from "./scene";
import { Viewport, type CameraLink } from "./Viewport";
import { labels, type Case, type Kernel, type Manifest, type Outcome, type Result } from "./types";

const kernels: Kernel[] = ["occt", "ngk"];
const number = (value: number | null | undefined) => value == null ? "—" : new Intl.NumberFormat("en-GB", { maximumFractionDigits: 5 }).format(value);
const meshUrl = (result: Result, run: string | null) => result.meshUrl ? `${result.meshUrl}?run=${encodeURIComponent(run ?? "")}` : null;

function Badge({ outcome }: { outcome: Outcome }) {
  return <span className={`badge ${outcome}`}><i />{labels[outcome] ?? outcome}</span>;
}

/** Render a still from the genuine mesh when its card approaches the viewport. */
function Thumbnail({ url, kernel, outcome }: { url: string | null; kernel: Kernel; outcome: Outcome }) {
  const host = useRef<HTMLDivElement>(null);
  const [image, setImage] = useState("");
  const [error, setError] = useState(false);
  useEffect(() => {
    if (!url) return;
    let active = true;
    const observer = new IntersectionObserver(entries => {
      if (!entries.some(entry => entry.isIntersecting)) return;
      observer.disconnect();
      preview(url, colors[kernel]).then(value => { if (active) setImage(value); })
        .catch(() => { if (active) setError(true); });
    }, { rootMargin: "350px" });
    observer.observe(host.current!);
    return () => { active = false; observer.disconnect(); };
  }, [url, kernel]);
  return <div className={`thumbnail ${kernel}`} ref={host}>
    <span className="kernel-label">{kernel === "occt" ? "OCP · reference" : "NGK"}</span>
    {image ? <img src={image} alt={`Preview ${kernel === "occt" ? "OCCT via OCP" : "NGK"}`} />
      : <div className="thumbnail-placeholder">{!url ? <><span className="no-model">∅</span>{labels[outcome]}</>
        : error ? "Preview unavailable" : <><span className="preview-spinner" />Preparing preview</>}</div>}
    <span className={`status-dot ${outcome}`} title={labels[outcome]} />
  </div>;
}

function Metrics({ result, item }: { result: Result; item: Case }) {
  const delta = (value: number | null | undefined, expected: number | undefined) => {
    if (value == null || expected == null) return null;
    if (expected === 0) return value === 0 ? "no deviation" : `deviation ${number(value)}`;
    const relative = (value - expected) / Math.abs(expected) * 100;
    return Math.abs(relative) < 0.0001 ? "deviation < 0.0001%" : `deviation ${relative > 0 ? "+" : ""}${number(relative)} %`;
  };
  return <div className="metrics">
    <div><span>Volume</span><strong>{number(result.volume)}</strong><small>{delta(result.volume, item.expect?.volume.value)}</small></div>
    <div><span>Area</span><strong>{number(result.area)}</strong><small>{delta(result.area, item.expect?.area.value)}</small></div>
    <div><span>Triangles</span><strong>{number(result.mesh?.triangles)}</strong><small>{result.mesh ? result.mesh.closed ? "closed mesh" : `${number(result.mesh.unmatched_edges)} unmatched edges` : "no mesh"}</small></div>
  </div>;
}

function Comparison({ item, data, onClose, onStep, canPrevious, canNext }: {
  item: Case; data: Manifest; onClose: () => void; onStep: (direction: number) => void;
  canPrevious: boolean; canNext: boolean;
}) {
  const [geometries, setGeometries] = useState<Partial<Record<Kernel, THREE.BufferGeometry>>>({});
  const [errors, setErrors] = useState<Partial<Record<Kernel, string>>>({});
  const [loaded, setLoaded] = useState<Partial<Record<Kernel, boolean>>>({});
  const [wireframe, setWireframe] = useState(false);
  const [linked, setLinked] = useState(true);
  const [reset, setReset] = useState(0);
  const dialog = useRef<HTMLDialogElement>(null);
  const link = useMemo<CameraLink>(() => ({ enabled: true, listeners: new Map() }), []);
  link.enabled = linked;
  useEffect(() => {
    const element = dialog.current!;
    element.showModal();
    const previous = document.body.style.overflow;
    document.body.style.overflow = "hidden";
    return () => { document.body.style.overflow = previous; };
  }, []);
  useEffect(() => {
    const controller = new AbortController();
    const created: THREE.BufferGeometry[] = [];
    let active = true;
    for (const kernel of kernels) {
      const url = meshUrl(item.results[kernel], data.kernels[kernel].run);
      if (!url) { setLoaded(previous => ({ ...previous, [kernel]: true })); continue; }
      loadMesh(url, controller.signal).then(geometry => {
        if (!active) { geometry.dispose(); return; }
        created.push(geometry);
        setGeometries(previous => ({ ...previous, [kernel]: geometry }));
      }).catch(error => {
        if (active) setErrors(previous => ({ ...previous, [kernel]: String(error) }));
      }).finally(() => { if (active) setLoaded(previous => ({ ...previous, [kernel]: true })); });
    }
    return () => { active = false; controller.abort(); created.forEach(geometry => geometry.dispose()); };
  }, [item, data]);
  const bounds = useMemo(() => {
    const box = new THREE.Box3();
    for (const geometry of Object.values(geometries)) if (geometry.boundingBox) box.union(geometry.boundingBox);
    return box;
  }, [geometries]);
  const ready = kernels.every(kernel => loaded[kernel]);
  return <dialog ref={dialog} className="comparison" onCancel={event => { event.preventDefault(); onClose(); }} aria-label={`3D comparison: ${item.title}`}>
    <header className="viewer-header"><div><span className="eyebrow">{item.category} / 3D comparison</span><h2>{item.title}</h2></div>
      <div className="viewer-actions"><button disabled={!canPrevious} onClick={() => onStep(-1)} aria-label="Previous case">←</button><button disabled={!canNext} onClick={() => onStep(1)} aria-label="Next case">→</button><button onClick={onClose} className="close-button" aria-label="Close viewer">Close <kbd>Esc</kbd></button></div>
    </header>
    <div className="viewer-toolbar"><div className="control-group"><button aria-pressed={linked} className={linked ? "active" : ""} onClick={() => setLinked(!linked)}>◎ Linked cameras</button><button aria-pressed={wireframe} className={wireframe ? "active" : ""} onClick={() => setWireframe(!wireframe)}>▧ Wireframe</button><button onClick={() => setReset(value => value + 1)}>↺ Reset view</button></div><span>Drag to rotate · Scroll to zoom · Right-drag to pan</span></div>
    <div className="viewer-panes">{kernels.map(kernel => {
      const result = item.results[kernel];
      return <section className={`viewer-pane ${kernel}`} key={kernel}>
        <div className="pane-title"><strong>{kernel === "occt" ? "OCCT / OCP" : "NGK"}<small>{kernel === "occt" ? "Benchmark reference" : "Development kernel"}</small></strong><Badge outcome={result.outcome} /></div>
        <div className="canvas-area">{ready && geometries[kernel] ? <Viewport geometry={geometries[kernel]!} bounds={bounds} kernel={kernel} wireframe={wireframe} reset={reset} link={link} />
          : <div className="empty-state">{result.meshUrl ? errors[kernel] || "Loading 3D model…" : <><span className="no-model">∅</span><strong>No model produced</strong><p>{result.why || labels[result.outcome]}</p></>}</div>}
          {result.meshUrl && <a className="download" href={result.meshUrl} download>Download STL ↗</a>}
        </div>
        <Metrics result={result} item={item} />
        {result.why && <details className="diagnostic" open={result.outcome !== "pass" && result.outcome !== "built"}><summary>Benchmark diagnostic</summary><p>{result.why}</p></details>}
      </section>;
    })}</div>
    <footer className="case-description"><span className="eyebrow">Case definition · {item.id}</span><p>{item.description}</p>
      {item.expect && <div className="expected">Expected values: volume <strong>{number(item.expect.volume.value)}</strong> · area <strong>{number(item.expect.area.value)}</strong></div>}
    </footer>
  </dialog>;
}

export default function App() {
  const [data, setData] = useState<Manifest | null>(null);
  const [error, setError] = useState("");
  const [query, setQuery] = useState("");
  const [category, setCategory] = useState("all");
  const [outcome, setOutcome] = useState("all");
  const [selected, setSelected] = useState(new URLSearchParams(window.location.search).get("case"));
  const load = async () => {
    try {
      const response = await fetch(`manifest.json?t=${Date.now()}`, { cache: "no-store" });
      if (!response.ok) throw new Error("Benchmark data is unavailable. Please try again later.");
      const manifest: Manifest = await response.json();
      if (!Array.isArray(manifest.cases)) throw new Error("The benchmark manifest is invalid.");
      setData(manifest); setError("");
    } catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)); }
  };
  useEffect(() => { void load(); }, []);
  const select = (id: string | null) => {
    setSelected(id);
    const url = new URL(window.location.href);
    if (id) url.searchParams.set("case", id); else url.searchParams.delete("case");
    window.history.replaceState(null, "", url);
  };
  const filtered = useMemo(() => (data?.cases ?? []).filter(item =>
    (category === "all" || item.category === category) &&
    (outcome === "all" || (outcome === "mesh" ? item.results.ngk.meshUrl : item.results.ngk.outcome === outcome)) &&
    `${item.id} ${item.title}`.toLowerCase().includes(query.toLowerCase())), [data, query, category, outcome]);
  const item = data?.cases.find(candidate => candidate.id === selected);
  const index = filtered.findIndex(candidate => candidate.id === selected);
  if (!data) return <div className="startup"><span className="brand-mark">N</span><h1>Benchmark explorer</h1><p>{error || "Loading models…"}</p>{error && <button onClick={() => void load()}>Retry</button>}</div>;
  const tally = data.kernels.ngk.tally;
  const meshCount = (kernel: Kernel) => data.cases.filter(candidate => candidate.results[kernel].meshUrl).length;
  return <>
    <header className="topbar"><a className="brand" href="./"><span className="brand-mark">N</span><strong>NGK<span> / benchmark</span></strong></a><div className="topbar-meta"><span className="live-dot" />{data.cases.length} cases · OCCT / OCP reference<button onClick={() => void load()} title="Reload exported results">↻ Refresh</button></div></header>
    <main>
      <section className="intro"><div><span className="eyebrow">BREP BENCH / VISUAL EXPLORER</span><h1>The models, side by side.</h1><p>Browse reference models alongside their NGK results.<br />Click any card to open the 3D comparison.</p></div><div className="coverage"><div><span className="reference-dot" />OCCT / OCP<strong>{meshCount("occt")}<small> models</small></strong></div><div><span className="ngk-dot" />NGK<strong>{meshCount("ngk")}<small> models</small></strong></div></div></section>
      <section className="scoreboard" aria-label="NGK results">{(["pass", "wrong", "refused", "crashed"] as Outcome[]).map(status => <button key={status} className={outcome === status ? "selected" : ""} onClick={() => setOutcome(outcome === status ? "all" : status)}><span className={`status-dot ${status}`} /><strong>{tally[status] ?? 0}</strong><span>{status === "pass" ? "passing cases" : status === "wrong" ? "incorrect results" : status === "refused" ? "refused cases" : "crashes"}</span></button>)}</section>
      <section className="gallery-toolbar"><div className="search"><span>⌕</span><input aria-label="Search models" placeholder="Search for a part or operation…" value={query} onChange={event => setQuery(event.target.value)} /></div><select aria-label="Category" value={category} onChange={event => setCategory(event.target.value)}><option value="all">All categories</option>{[...new Set(data.cases.map(candidate => candidate.category))].sort().map(value => <option key={value} value={value}>{value}</option>)}</select><select aria-label="NGK result" value={outcome} onChange={event => setOutcome(event.target.value)}><option value="all">All NGK results</option><option value="mesh">With an NGK model</option>{Object.entries(labels).map(([value, label]) => <option value={value} key={value}>{label}</option>)}</select><span className="case-count">{filtered.length} / {data.cases.length} cases</span></section>
      {error && <p role="alert" className="load-error">{error}</p>}
      <div className="gallery">{filtered.map(candidate => <button className="model-card" key={candidate.id} onClick={() => select(candidate.id)} aria-label={`Open ${candidate.title} in 3D`}>
        <div className="card-previews">{kernels.map(kernel => <Thumbnail key={`${kernel}:${data.kernels[kernel].run}`} kernel={kernel} url={meshUrl(candidate.results[kernel], data.kernels[kernel].run)} outcome={candidate.results[kernel].outcome} />)}<span className="open-hint">Explore in 3D ↗</span></div>
        <div className="card-info"><span className="category">{candidate.category}</span><h2>{candidate.title}</h2><div className="card-bottom"><code>{candidate.id.split("/")[1]}</code><Badge outcome={candidate.results.ngk.outcome} /></div></div>
      </button>)}</div>
      {!filtered.length && <div className="no-results"><h2>No matching cases.</h2><button onClick={() => { setQuery(""); setCategory("all"); setOutcome("all"); }}>Reset filters</button></div>}
      <footer className="page-footer"><span>Meshes produced by {data.kernels.occt.version ?? "OCCT / OCP not run"} and {data.kernels.ngk.version ?? "NGK not run"}</span><span>NGK run: {data.kernels.ngk.run ? new Date(data.kernels.ngk.run!).toLocaleString("en-GB") : "not run"} · <a href="manifest.json" download>JSON data ↗</a></span></footer>
    </main>
    {item && <Comparison key={`${item.id}:${data.generatedAt}`} item={item} data={data} onClose={() => select(null)} onStep={direction => { const next = filtered[index + direction]; if (next) select(next.id); }} canPrevious={index > 0} canNext={index >= 0 && index < filtered.length - 1} />}
  </>;
}
