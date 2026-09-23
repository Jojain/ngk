const SHARED_CODE_PARAM = "zc";
const MAX_SHARE_URL_LENGTH = 8_000;

function toBase64Url(bytes: Uint8Array): string {
  let binary = "";
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return btoa(binary).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/g, "");
}

function fromBase64Url(value: string): Uint8Array {
  const padding = "=".repeat((4 - (value.length % 4)) % 4);
  const base64 = value.replace(/-/g, "+").replace(/_/g, "/") + padding;
  const binary = atob(base64);
  return Uint8Array.from(binary, (character) => character.charCodeAt(0));
}

export async function compressCode(code: string): Promise<string> {
  const stream = new Blob([new TextEncoder().encode(code)])
    .stream()
    .pipeThrough(new CompressionStream("deflate"));
  const bytes = new Uint8Array(await new Response(stream).arrayBuffer());
  return toBase64Url(bytes);
}

export async function decompressCode(value: string): Promise<string> {
  const bytes = fromBase64Url(value);
  const stream = new Blob([bytes.buffer as ArrayBuffer])
    .stream()
    .pipeThrough(new DecompressionStream("deflate"));
  return new Response(stream).text();
}

export async function codeFromUrl(): Promise<string | null> {
  const value = new URL(window.location.href).searchParams.get(SHARED_CODE_PARAM);
  if (!value) return null;
  try {
    return await decompressCode(value);
  } catch {
    return null;
  }
}

export async function shareCode(code: string): Promise<"copied" | "too-long" | "unavailable"> {
  const url = new URL(window.location.href);
  const compressed = await compressCode(code);
  url.searchParams.set(SHARED_CODE_PARAM, compressed);
  if (url.toString().length > MAX_SHARE_URL_LENGTH) return "too-long";
  window.history.replaceState({}, "", url);
  if (!navigator.clipboard) return "unavailable";
  await navigator.clipboard.writeText(url.toString());
  return "copied";
}
