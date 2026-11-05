import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { fileURLToPath } from "node:url";

// This page consumes native benchmark artifacts and has no WASM build dependency.
export default defineConfig({
  base: process.env.VITE_BASE_PATH ?? "/",
  root: fileURLToPath(new URL("./benchmark", import.meta.url)),
  publicDir: fileURLToPath(new URL("../brep-bench/viewer-data", import.meta.url)),
  plugins: [react()],
  server: { host: "127.0.0.1", port: 3942, strictPort: true },
  build: { outDir: "../benchmark-dist", emptyOutDir: true },
});
