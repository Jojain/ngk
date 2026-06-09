import { defineConfig } from "vite";
import type { Plugin } from "vite";
import react from "@vitejs/plugin-react";
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";

declare const process: {
  env: Record<string, string | undefined>;
};

function ngkDebugViewerPlugin(): Plugin {
  const dumps: unknown[] = [];
  const endpoint = "/__ngk_debug/dumps";

  return {
    name: "ngk-debug-viewer",
    configureServer(server) {
      server.middlewares.use(endpoint, async (req, res) => {
        if (req.method === "GET") {
          sendJson(res, 200, dumps);
          return;
        }
        if (req.method === "DELETE") {
          dumps.length = 0;
          sendJson(res, 200, []);
          return;
        }
        if (req.method === "POST") {
          try {
            const body = await readBody(req);
            const payload = JSON.parse(body);
            dumps.push({
              receivedAt: new Date().toISOString(),
              sequence: dumps.length + 1,
              payload,
            });
            sendJson(res, 204, null);
          } catch (error) {
            sendJson(res, 400, {
              error: error instanceof Error ? error.message : String(error),
            });
          }
          return;
        }
        res.statusCode = 405;
        res.end();
      });
    },
  };
}

function readBody(req: {
  setEncoding: (encoding: string) => void;
  on: (event: string, callback: (chunk?: string) => void) => void;
}) {
  return new Promise<string>((resolve, reject) => {
    let body = "";
    req.setEncoding("utf8");
    req.on("data", (chunk) => {
      body += chunk ?? "";
    });
    req.on("end", () => resolve(body));
    req.on("error", () => reject(new Error("failed to read request body")));
  });
}

function sendJson(
  res: {
    statusCode: number;
    setHeader: (name: string, value: string) => void;
    end: (body?: string) => void;
  },
  status: number,
  value: unknown,
) {
  res.statusCode = status;
  res.setHeader("Access-Control-Allow-Origin", "*");
  if (status === 204) {
    res.end();
    return;
  }
  res.setHeader("Content-Type", "application/json");
  res.end(JSON.stringify(value));
}

export default defineConfig({
  base: process.env.VITE_BASE_PATH ?? "/",
  plugins: [react(), wasm(), topLevelAwait(), ngkDebugViewerPlugin()],
  server: {
    fs: {
      allow: [".."],
    },
  },
  optimizeDeps: {
    exclude: ["./src/wasm/ngk.js"],
  },
});
