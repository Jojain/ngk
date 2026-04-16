import { useEffect, useState } from "react";
import initWasm, * as ngk from "../wasm/ngk";

export type Kernel = typeof ngk;

let kernelPromise: Promise<Kernel> | null = null;

export function loadKernel(): Promise<Kernel> {
  if (!kernelPromise) {
    kernelPromise = initWasm().then(() => ngk);
  }
  return kernelPromise;
}

export function useKernel(): Kernel | null {
  const [kernel, setKernel] = useState<Kernel | null>(null);
  useEffect(() => {
    let cancelled = false;
    loadKernel().then((k) => {
      if (!cancelled) setKernel(k);
    });
    return () => {
      cancelled = true;
    };
  }, []);
  return kernel;
}
