import wasmTypes from "./wasm/ngk.d.ts?raw";
import scriptTypes from "../examples/ngk-script.d.ts?raw";

export const ngkModuleTypes = `declare module "ngk-wasm" {\n${wasmTypes}\n}`;

export { scriptTypes };
