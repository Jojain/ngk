import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";

declare const process: {
  env: Record<string, string | undefined>;
};

export default defineConfig({
  base: process.env.VITE_BASE_PATH ?? "/",
  plugins: [react(), wasm(), topLevelAwait()],
  server: {
    fs: {
      allow: [".."],
    },
  },
  optimizeDeps: {
    exclude: ["./src/wasm/ngk.js"],
  },
});
