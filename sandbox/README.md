# NGK Sandbox

The editor runs TypeScript or JavaScript in a Web Worker against NGK's WASM
bindings. Call `show(shape)` to send a model, solid, face, or edge to the shared
viewer. The complete threaded bolt is the default example.

Use `Share` to copy the current script as a compressed `zc` URL. Opening that
URL restores the script before any locally saved draft.

From the NGK repository root, run `npm ci`, then `npm run dev --workspace sandbox`.
The production build is `npm run build --workspace sandbox` with
`VITE_BASE_PATH=/ngk/sandbox/` for GitHub Pages.

The live site is published from the NGK repository at
`https://jojain.github.io/ngk/sandbox/`. The root Pages workflow builds both the
visualization and this sandbox into one Pages artifact.
