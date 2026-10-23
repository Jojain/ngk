import { useEffect, useRef, useState } from "react";
import type { PointerEvent as ReactPointerEvent } from "react";
import { useKernel } from "../../kernel/useKernel";
import {
  choices, curvePoints, empty, execute, points, preset, refKey, refName, relations, shapeName,
  type Command, type Constraint, type Geometry, type Preset, type Ref, type RefType,
  type RelationKind, type Session, type Shape, type XY,
} from "./model";
import "./style.css";

type Tool = "select" | "point" | "segment" | "circle" | "arc";
const tools: [Tool, string][] = [["select", "Déplacer"], ["point", "Point"], ["segment", "Segment"], ["circle", "Cercle"], ["arc", "Arc"]];
const fmt = (x: number) => Number(x.toFixed(3)).toString();
const polyline = (p: XY[]) => p.map(([x, y]) => `${x},${-y}`).join(" ");

export default function Solver2d() {
  const kernel = useKernel();
  const session = useRef<Session | null>(null);
  const svg = useRef<SVGSVGElement>(null);
  const inspector = useRef<HTMLElement>(null);
  const [view, setView] = useState(empty);
  const [error, setError] = useState("");
  const [elapsed, setElapsed] = useState(0);
  const [tool, setTool] = useState<Tool>("select");
  const [draft, setDraft] = useState<XY[]>([]);
  const [cursor, setCursor] = useState<XY | null>(null);
  const [clockwise, setClockwise] = useState(false);
  const [snap, setSnap] = useState(false);
  const [auto, setAuto] = useState(true);
  const [kind, setKind] = useState<RelationKind>("coincident");
  const [selection, setSelection] = useState<Ref[]>([]);
  const [dimension, setDimension] = useState("10");
  const [editing, setEditing] = useState<number | null>(null);
  const [box, setBox] = useState([-60, -40, 120, 80]);
  const [target, setTarget] = useState<XY | null>(null);
  const drag = useRef<{ point: Ref; client: XY; moved: boolean } | null>(null);
  const pending = useRef<{ point: Ref; position: XY } | null>(null);
  const frame = useRef<number | null>(null);
  const spec: { label: string; refs: RefType[]; unit?: string } = relations[kind];

  function cancelDrag() {
    if (frame.current !== null) cancelAnimationFrame(frame.current);
    frame.current = null; pending.current = null; drag.current = null;
  }
  function send(command: Command, solveAfter = false) {
    if (!session.current) return;
    try {
      const started = performance.now();
      let next = execute(session.current, command);
      if (solveAfter && auto) next = execute(session.current, { op: "solve" });
      setElapsed(performance.now() - started); setView(next); setError("");
      if (command.op !== "solve" || !command.target) setTarget(null);
      return next;
    } catch (e) { setError(String(e)); }
  }
  function load(name: Preset) {
    if (!kernel) return;
    cancelDrag(); session.current?.free(); session.current = new kernel.SolverPlayground();
    try {
      const started = performance.now(); setView(preset(session.current, name));
      setElapsed(performance.now() - started); setError("");
    } catch (e) { setView(empty); setError(String(e)); }
    setDraft([]); setSelection([]); setEditing(null); setTool("select"); setTarget(null); setBox([-60, -40, 120, 80]);
    setKind("coincident"); setDimension("10"); setCursor(null);
    if (inspector.current) inspector.current.scrollTop = 0;
  }
  useEffect(() => {
    if (!kernel) return;
    if (!kernel.SolverPlayground) {
      setError("Le noyau chargé ne contient pas le playground. Reconstruisez WebAssembly puis rechargez la page.");
      return;
    }
    load("rectangle");
    return () => { cancelDrag(); session.current?.free(); session.current = null; };
  }, [kernel]);

  function position(e: ReactPointerEvent): XY {
    const matrix = svg.current!.getScreenCTM()!;
    const p = new DOMPoint(e.clientX, e.clientY).matrixTransform(matrix.inverse());
    const quantize = (v: number) => snap ? Math.round(v) : v;
    return [quantize(p.x), quantize(-p.y)];
  }
  function pick(r: Ref) {
    setSelection((old) => {
      if (old.some((p) => refKey(p) === refKey(r))) return old;
      return old.length < spec.refs.length ? [...old, r] : [r];
    });
    setEditing(null);
  }
  function startDrag(e: ReactPointerEvent, r: Ref) {
    if (tool !== "select" || e.button !== 0) return;
    e.stopPropagation(); pick(r);
    drag.current = { point: r, client: [e.clientX, e.clientY], moved: false };
    svg.current!.setPointerCapture(e.pointerId);
  }
  function flush() {
    frame.current = null;
    const next = pending.current; pending.current = null;
    if (next) { setTarget(next.position); send({ op: "solve", target: next }); }
  }
  function move(e: ReactPointerEvent) {
    const p = position(e); setCursor(p);
    if (!drag.current) return;
    if (!drag.current.moved && Math.hypot(e.clientX - drag.current.client[0], e.clientY - drag.current.client[1]) < 3) return;
    drag.current.moved = true; pending.current = { point: drag.current.point, position: p };
    if (frame.current === null) frame.current = requestAnimationFrame(flush);
  }
  function finish(e: ReactPointerEvent) {
    if (drag.current?.moved) {
      pending.current = { point: drag.current.point, position: position(e) };
      if (frame.current !== null) cancelAnimationFrame(frame.current);
      flush();
    }
    drag.current = null;
    if (svg.current?.hasPointerCapture(e.pointerId)) svg.current.releasePointerCapture(e.pointerId);
  }
  function draw(e: ReactPointerEvent) {
    if (e.button !== 0) return;
    if (tool === "select") { setSelection([]); setEditing(null); return; }
    const p = position(e);
    if (tool === "point") { send({ op: "add_geometry", geometry: { kind: "point", position: p } }, true); return; }
    const pts = [...draft, p];
    if (pts.length < (tool === "arc" ? 3 : 2)) { setDraft(pts); return; }
    const radius = Math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]);
    if (radius < 1e-5) { setError("Placez deux points distincts."); return; }
    let geometry: Geometry;
    if (tool === "segment") geometry = { kind: "segment", start: pts[0], end: pts[1] };
    else if (tool === "circle") geometry = { kind: "circle", center: pts[0], radius };
    else {
      const start_angle = Math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]);
      const end = Math.atan2(p[1] - pts[0][1], p[0] - pts[0][0]);
      const tau = Math.PI * 2;
      const sweep = clockwise ? -((start_angle - end + tau) % tau) : (end - start_angle + tau) % tau;
      if (Math.abs(sweep) < 1e-5) { setError("L’arc doit avoir une ouverture non nulle."); return; }
      geometry = { kind: "arc", center: pts[0], radius, start_angle, sweep };
    }
    if (send({ op: "add_geometry", geometry }, true)) setDraft([]);
  }
  function addRelation() {
    const value = spec.unit ? Number(dimension) : undefined;
    if (spec.unit && (!dimension.trim() || !Number.isFinite(value))) { setError("Saisissez une valeur finie."); return; }
    const relation = { kind, a: selection[0], b: selection[1], c: selection[2], value };
    if (send(editing === null ? { op: "add_constraint", relation } : { op: "update_constraint", id: editing, relation }, true)) {
      setEditing(null); setSelection([]);
    }
  }
  function editConstraint(c: Constraint) {
    setKind(c.relation.kind); setSelection([c.relation.a, c.relation.b, c.relation.c].filter((r): r is Ref => !!r));
    setDimension(String(c.relation.value ?? 10)); setEditing(c.id); setTool("select"); setDraft([]);
  }
  function fit() {
    const all = view.geometry.flatMap((s) => [...points(s).map((p) => p.position), ...curvePoints(s)]);
    if (!all.length) { setBox([-60, -40, 120, 80]); return; }
    const xs = all.map((p) => p[0]), ys = all.map((p) => -p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const w = Math.max(30, maxX - minX + 20), h = Math.max(30, maxY - minY + 20);
    setBox([(minX + maxX - w) / 2, (minY + maxY - h) / 2, w, h]);
  }
  function zoom(factor: number) {
    setBox(([x, y, w, h]) => [x + w * (1 - factor) / 2, y + h * (1 - factor) / 2, w * factor, h * factor]);
  }
  const handleSize = box[2] / 180;
  const selectedIds = new Set(selection.map((r) => r.id));
  const suspectIds = new Set(view.constraints.filter((c) => c.suspect).flatMap((c) => [c.relation.a.id, c.relation.b?.id, c.relation.c?.id]));
  const validSelection = spec.refs.every((type, i) => selection[i] && choices(view.geometry, type).some((r) => refKey(r) === refKey(selection[i])));
  const hints: Record<Tool, string> = {
    select: "Glissez un point pour déplacer. Cliquez une courbe ou utilisez les références A / B pour la contraindre.",
    point: "Cliquez pour placer un point.", segment: draft.length ? "Placez la seconde extrémité." : "Placez la première extrémité.",
    circle: draft.length ? "Cliquez pour définir le rayon." : "Placez le centre du cercle.",
    arc: ["Placez le centre de l’arc.", "Placez son point de départ.", "Choisissez la direction de son extrémité."][draft.length],
  };

  return <div className="solver-app">
    <header className="solver-header"><div><span className="solver-eyebrow">ATELIER DE CONTRAINTES</span><h1>Solveur 2D</h1></div>
      <div className="solver-presets">{([["rectangle", "Rectangle"], ["circle", "Point sur cercle"], ["arc", "Arc tangent"], ["empty", "Vider"]] as [Preset, string][]).map(([id, label]) =>
        <button key={id} onClick={() => load(id)}>{label}</button>)}</div>
    </header>
    <div className="solver-workspace">
      <main className="solver-stage">
        <div className="solver-toolbar">{tools.map(([id, label]) => <button key={id} aria-pressed={tool === id} onClick={() => { cancelDrag(); setTool(id); setDraft([]); setTarget(null); }}>{label}</button>)}
          <label><input type="checkbox" checked={snap} onChange={(e) => setSnap(e.target.checked)} /> Grille</label>
          {tool === "arc" && <label><input type="checkbox" checked={clockwise} onChange={(e) => setClockwise(e.target.checked)} /> Horaire</label>}
          {!!draft.length && <button onClick={() => setDraft([])}>Annuler</button>}
        </div>
        <div className="solver-hint">{hints[tool]}</div>
        <svg ref={svg} className={`solver-canvas ${tool === "select" ? "" : "drawing"}`} viewBox={box.join(" ")} aria-label="Plan de dessin du solveur"
          onPointerDown={draw} onPointerMove={move} onPointerUp={finish} onPointerCancel={() => { cancelDrag(); setTarget(null); }}>
          <defs><pattern id="solver-grid" width="5" height="5" patternUnits="userSpaceOnUse"><path d="M 5 0 L 0 0 0 5" fill="none" stroke="#263344" strokeWidth="0.12" /></pattern></defs>
          <rect x={box[0]} y={box[1]} width={box[2]} height={box[3]} fill="url(#solver-grid)" />
          <path d={`M ${box[0]} 0 H ${box[0] + box[2]} M 0 ${box[1]} V ${box[1] + box[3]}`} stroke="#44526a" strokeWidth="0.18" />
          {view.geometry.map((shape) => <g key={shape.id} className={suspectIds.has(shape.id) ? "suspect" : selectedIds.has(shape.id) ? "selected" : ""}>
            {shape.kind !== "point" && <>
              <polyline points={polyline(curvePoints(shape))} className="solver-curve" />
              <polyline points={polyline(curvePoints(shape))} className="solver-hit" onPointerDown={(e) => { if (tool === "select") { e.stopPropagation(); pick({ id: shape.id, role: "whole" }); } }} />
            </>}
            {points(shape).map(({ ref: r, position: p }) => <circle key={r.role} cx={p[0]} cy={-p[1]} r={handleSize * (r.role === "center" ? 0.7 : 1)}
              className={`solver-point ${selection.some((s) => refKey(s) === refKey(r)) ? "picked" : ""}`} onPointerDown={(e) => startDrag(e, r)}>
              <title>{refName(r, view.geometry)}</title>
            </circle>)}
            <text x={points(shape)[0].position[0] + handleSize * 1.7} y={-points(shape)[0].position[1] - handleSize * 1.7} fontSize={handleSize * 2.4} className="solver-label">{shape.id}</text>
          </g>)}
          {draft.length > 0 && cursor && <polyline className="solver-preview" points={polyline([...draft, cursor])} />}
          {draft.map((p, i) => <circle key={i} cx={p[0]} cy={-p[1]} r={handleSize} fill="#ffd18a" />)}
          {target && <g className="solver-target"><path d={`M ${target[0] - handleSize * 2} ${-target[1]} h ${handleSize * 4} M ${target[0]} ${-target[1] - handleSize * 2} v ${handleSize * 4}`} /><circle cx={target[0]} cy={-target[1]} r={handleSize * 2} /></g>}
        </svg>
        <footer className="solver-stage-footer"><span>{cursor ? `x ${fmt(cursor[0])} · y ${fmt(cursor[1])}` : "Unités libres · angles en degrés"}</span>
          <div><button aria-label="Dézoomer" onClick={() => zoom(1.3)}>−</button><button aria-label="Zoomer" onClick={() => zoom(1 / 1.3)}>+</button><button onClick={fit}>Cadrer</button></div>
        </footer>
      </main>
      <aside className="solver-inspector" ref={inspector}>
        <section className={`solver-status ${view.status}`} aria-live="polite">
          <strong>{!view.geometry.length ? "Aucune primitive" : view.status === "error" ? "À résoudre" : view.status === "edited" ? "Modifications à résoudre" : view.dof === 0 ? "Entièrement contraint" : "Sous-contraint · résolu"}</strong>
          <div className="solver-metrics"><span><b>{view.dof ?? "—"}</b> degrés de liberté estimés</span><span><b>{elapsed.toFixed(1)}</b> ms</span></div>
          {view.message && <p>{view.message}</p>}
          {view.constraints.some((c) => c.warning) && <p className="solver-warning">Dépendances détectées à cette configuration. Les contraintes sont satisfaites ; cela ne signifie pas qu’une relation peut être supprimée.</p>}
          {view.target_distance !== null && <p>Écart à la cible : {fmt(view.target_distance)} u. Les contraintes restent prioritaires.</p>}
          <div className="solver-solve-row"><button className="primary" onClick={() => send({ op: "solve" })}>Résoudre</button><label><input type="checkbox" checked={auto} onChange={(e) => setAuto(e.target.checked)} /> Après chaque édition</label></div>
        </section>
        {error && <div className="solver-error" role="alert">{error}</div>}
        <section><h2>{editing === null ? "Ajouter une contrainte" : `Modifier la contrainte ${editing}`}</h2>
          <label>Relation<select value={kind} onChange={(e) => { setKind(e.target.value as RelationKind); setSelection([]); setEditing(null); }}>{Object.entries(relations).map(([k, value]) => <option key={k} value={k}>{value.label}</option>)}</select></label>
          {spec.refs.map((type, i) => <label key={i}>Référence {"ABC"[i]}<select aria-label={`Référence ${"ABC"[i]}`} value={selection[i] ? refKey(selection[i]) : ""} onChange={(e) => {
            const choice = choices(view.geometry, type).find((r) => refKey(r) === e.target.value);
            setSelection((old) => { const next = [...old]; if (choice) next[i] = choice; else next.splice(i, 1); return next; });
          }}><option value="">Choisir…</option>{choices(view.geometry, type).map((r) => <option key={refKey(r)} value={refKey(r)}>{refName(r, view.geometry)}</option>)}</select></label>)}
          {spec.unit && <label>Valeur ({spec.unit})<input aria-label="Valeur de la contrainte" type="number" step="any" value={dimension} onChange={(e) => setDimension(e.target.value)} /></label>}
          <div className="solver-actions"><button className="primary" disabled={!validSelection} onClick={addRelation}>{editing === null ? "Ajouter" : "Appliquer"}</button>
            {(editing !== null || selection.length > 0) && <button onClick={() => { setEditing(null); setSelection([]); }}>Annuler</button>}</div>
        </section>
        <section><h2>Contraintes <span>{view.constraints.length}</span></h2>
          {!view.constraints.length && <p className="muted">Ajoutez une relation pour guider la géométrie.</p>}
          <div className="solver-list">{view.constraints.map((c) => <div key={c.id} className={`solver-item ${c.suspect ? "suspect" : c.warning ? "warning" : ""}`}>
            <button className="solver-item-main" onClick={() => editConstraint(c)}><strong>{relations[c.relation.kind].label}{c.relation.value != null ? ` · ${fmt(c.relation.value)}` : ""}</strong>
              <small>{[c.relation.a, c.relation.b, c.relation.c].filter((r): r is Ref => !!r).map((r) => refName(r, view.geometry)).join(" / ")}</small>
              {c.warning && <small>Dépendance détectée</small>}{c.suspect && <small>Groupe non résolu</small>}</button>
            <button className="solver-remove" aria-label={`Supprimer la contrainte ${c.id}`} onClick={() => { send({ op: "remove_constraint", id: c.id }, true); if (editing === c.id) setEditing(null); }}>×</button>
          </div>)}</div>
        </section>
        <section><h2>Primitives <span>{view.geometry.length}</span></h2><p className="muted">Supprimer une primitive retire aussi ses contraintes.</p>
          <div className="solver-list">{view.geometry.map((s: Shape) => <div className="solver-item" key={s.id}><button className="solver-item-main" onClick={() => pick({ id: s.id, role: s.kind === "point" ? "point" : "whole" })}>{shapeName(s)}</button><button aria-label={`Supprimer ${shapeName(s)}`} onClick={() => { send({ op: "remove_geometry", id: s.id }, true); setSelection([]); setEditing(null); }}>×</button></div>)}</div>
        </section>
      </aside>
    </div>
  </div>;
}
