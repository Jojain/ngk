import * as ts from "typescript";
import initWasm from "./wasm/ngk";
import { ngk, sceneFor, type Showable } from "./api";
import type { VizScene } from "@ngk/viewer";
import type { WorkerRequest, WorkerResponse } from "./protocol";

function send(message: WorkerResponse) {
  self.postMessage(message);
}

function format(value: unknown): string {
  if (value instanceof Error) return value.stack ?? value.message;
  if (typeof value === "string") return value;
  try {
    return JSON.stringify(value);
  } catch {
    return String(value);
  }
}

const output = {
  log: (...values: unknown[]) => send({ kind: "log", level: "log", text: values.map(format).join(" ") }),
  warn: (...values: unknown[]) => send({ kind: "log", level: "warn", text: values.map(format).join(" ") }),
  error: (...values: unknown[]) => send({ kind: "log", level: "error", text: values.map(format).join(" ") }),
};

const browserError = console.error.bind(console);
console.error = (...values: unknown[]) => {
  browserError(...values);
  output.error(...values);
};

function compile(code: string): string {
  const parsed = ts.createSourceFile("model.ts", code, ts.ScriptTarget.ES2022, true);
  if (ts.isExternalModule(parsed)) {
    throw new Error("Use the supplied ngk and show bindings; imports and exports are unavailable.");
  }
  const result = ts.transpileModule(code, {
    fileName: "model.ts",
    reportDiagnostics: true,
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
  });
  const errors = result.diagnostics?.filter((diagnostic) => diagnostic.category === ts.DiagnosticCategory.Error) ?? [];
  if (errors.length) {
    throw new Error(errors.map((diagnostic) => ts.flattenDiagnosticMessageText(diagnostic.messageText, "\n")).join("\n"));
  }
  return result.outputText;
}

self.onmessage = async (event: MessageEvent<WorkerRequest>) => {
  if (event.data.kind !== "run") return;
  try {
    send({ kind: "log", level: "log", text: "Loading NGK…" });
    await initWasm();
    send({ kind: "ready" });
    let shown = false;
    const show = (shape: Showable) => {
      const scene = sceneFor(shape) as VizScene;
      shown = true;
      send({ kind: "scene", scene });
    };
    const compiled = compile(event.data.code);
    const execute = new Function("ngk", "show", "console", `return (async () => {\n${compiled}\n})();`);
    await execute(ngk, show, output);
    send({ kind: "done", shown });
  } catch (error) {
    send({ kind: "error", text: format(error) });
  }
};
