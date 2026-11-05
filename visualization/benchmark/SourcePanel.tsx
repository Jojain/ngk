import { useEffect, useState } from "react";
import type { SourceFile } from "./types";

/** Load the real case implementation or one of its construction helpers on demand. */
export function SourcePanel({ files, revision }: { files: SourceFile[]; revision: string }) {
  const [open, setOpen] = useState(false);
  const [selected, setSelected] = useState(0);
  const [text, setText] = useState<string | null>(null);
  const [error, setError] = useState("");
  const [copyStatus, setCopyStatus] = useState("");
  const file = files[selected];
  useEffect(() => {
    if (!open || !file) return;
    const controller = new AbortController();
    setText(null); setError(""); setCopyStatus("");
    fetch(`${file.url}?revision=${encodeURIComponent(revision)}`, { signal: controller.signal })
      .then(async response => {
        if (!response.ok) throw new Error(`Source unavailable (${response.status}).`);
        return response.text();
      }).then(value => { if (!controller.signal.aborted) setText(value); })
      .catch(cause => { if (!controller.signal.aborted) setError(String(cause)); });
    return () => controller.abort();
  }, [open, file, revision]);
  const copy = async () => {
    try { await navigator.clipboard.writeText(text!); setCopyStatus("Copied"); }
    catch { setCopyStatus("Copy unavailable. Select the code or download the file."); }
  };
  return <details className="case-source" onToggle={event => setOpen(event.currentTarget.open)}>
    <summary>Source code <span>{file?.language === "rust" ? "Rust · NGK" : "Python · OCP"}</span></summary>
    {!file ? <p className="source-message">Source code is unavailable for this case.</p> : <>
      <div className="source-toolbar"><select aria-label="Source file" value={selected} onChange={event => setSelected(Number(event.target.value))}>
        {files.map((entry, index) => <option key={entry.path} value={index}>{entry.path}{index ? " · helper" : ""}</option>)}
      </select><button onClick={() => void copy()} disabled={text === null}>Copy</button><a href={file.url} download>Download ↗</a></div>
      {copyStatus && <p className="source-message" role="status">{copyStatus}</p>}
      {error ? <p className="source-message" role="alert">{error}</p> : text === null ? <p className="source-message">Loading source…</p>
        : <pre className="source-code" aria-label={`${file.language} source: ${file.path}`}><code>{text.trimEnd().split("\n").map((line, index) =>
          <span className="source-line" key={index}><span className="line-number" aria-hidden="true">{index + 1}</span><span>{line || " "}</span></span>)}</code></pre>}
    </>}
  </details>;
}
