import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

export const colors = { occt: 0xd9c294, ngk: 0x8ebcd2 };

/** Load the actual STL; a missing mesh remains an explicit unavailable result. */
export async function loadMesh(url: string, signal?: AbortSignal): Promise<THREE.BufferGeometry> {
  const response = await fetch(url, { signal });
  if (!response.ok) throw new Error(`Mesh unavailable (${response.status}).`);
  const geometry = new STLLoader().parse(await response.arrayBuffer());
  if (!geometry.attributes.position?.count) {
    geometry.dispose();
    throw new Error("The mesh is empty.");
  }
  geometry.computeBoundingBox();
  geometry.computeVertexNormals();
  return geometry;
}

export function lightScene(scene: THREE.Scene) {
  scene.add(new THREE.HemisphereLight(0xe2efff, 0x56545e, 2.3));
  const key = new THREE.DirectionalLight(0xffffff, 3.3);
  key.position.set(3, -4, 6);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xc6ddff, 1.4);
  fill.position.set(-4, 2, 2);
  scene.add(fill);
}

let previewRenderer: THREE.WebGLRenderer | undefined;
let queue: Promise<unknown> = Promise.resolve();
const previews = new Map<string, Promise<string>>();

/** One shared renderer produces still previews, avoiding hundreds of WebGL contexts. */
export function preview(url: string, color: number): Promise<string> {
  const key = `${url}:${color}`;
  const existing = previews.get(key);
  if (existing) return existing;
  const result = queue.then(async () => {
    const geometry = await loadMesh(url);
    const material = new THREE.MeshStandardMaterial({ color, roughness: 0.62, metalness: 0.12, side: THREE.DoubleSide });
    try {
      previewRenderer ??= new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true });
      const renderer = previewRenderer;
      renderer.setSize(360, 238, false);
      renderer.setPixelRatio(1);
      renderer.setClearColor(0x10171c, 0);
      const scene = new THREE.Scene();
      lightScene(scene);
      const bounds = geometry.boundingBox!;
      const center = bounds.getCenter(new THREE.Vector3());
      const radius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, 1e-8);
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.copy(center).negate();
      scene.add(mesh);
      const camera = new THREE.PerspectiveCamera(34, 360 / 238, radius / 1000, radius * 100);
      camera.up.set(0, 0, 1);
      camera.position.set(1.3, -1.7, 1.2).normalize().multiplyScalar(radius * 3.7);
      camera.lookAt(0, 0, 0);
      renderer.render(scene, camera);
      return renderer.domElement.toDataURL("image/png");
    } finally {
      geometry.dispose();
      material.dispose();
    }
  });
  previews.set(key, result);
  queue = result.catch(() => undefined);
  return result;
}
