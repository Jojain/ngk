import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { colors, lightScene } from "./scene";
import type { Kernel } from "./types";

type Pose = { position: THREE.Vector3; target: THREE.Vector3 };
export type CameraLink = { enabled: boolean; listeners: Map<Kernel, (pose: Pose) => void> };

/** An orbit/pan/zoom viewer sharing a common frame and optional camera pose with its peer. */
export function Viewport({ geometry, bounds, kernel, wireframe, reset, link }: {
  geometry: THREE.BufferGeometry; bounds: THREE.Box3; kernel: Kernel;
  wireframe: boolean; reset: number; link: CameraLink;
}) {
  const host = useRef<HTMLDivElement>(null);
  const materialRef = useRef<THREE.MeshStandardMaterial>();
  const [error, setError] = useState("");
  useEffect(() => {
    const container = host.current!;
    let renderer: THREE.WebGLRenderer;
    try { renderer = new THREE.WebGLRenderer({ antialias: true }); }
    catch { setError("This browser cannot open the WebGL viewer. You can still download the STL."); return; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x11191f);
    container.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    lightScene(scene);
    const center = bounds.getCenter(new THREE.Vector3());
    const radius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, 1e-8);
    const material = new THREE.MeshStandardMaterial({ color: colors[kernel], roughness: 0.63, metalness: 0.15, side: THREE.DoubleSide });
    materialRef.current = material;
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.copy(center).negate();
    scene.add(mesh);
    const gridSize = Math.pow(10, Math.ceil(Math.log10(radius * 3)));
    const grid = new THREE.GridHelper(gridSize, 20, 0x35434c, 0x202e37);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = bounds.min.z - center.z - radius * 0.015;
    scene.add(grid);
    const axes = new THREE.AxesHelper(radius * 0.5);
    axes.position.copy(grid.position);
    scene.add(axes);
    const camera = new THREE.PerspectiveCamera(35, 1, radius / 10000, radius * 1000);
    camera.up.set(0, 0, 1);
    camera.position.set(1.3, -1.7, 1.2).normalize().multiplyScalar(radius * 3.8);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.minDistance = radius * 0.015;
    controls.maxDistance = radius * 50;
    controls.update();
    let applying = false;
    link.listeners.set(kernel, pose => {
      applying = true;
      camera.position.copy(pose.position);
      controls.target.copy(pose.target);
      controls.update();
      applying = false;
    });
    const changed = () => {
      if (!link.enabled || applying) return;
      const pose = { position: camera.position.clone(), target: controls.target.clone() };
      for (const [other, listener] of link.listeners) if (other !== kernel) listener(pose);
    };
    controls.addEventListener("change", changed);
    const observer = new ResizeObserver(() => {
      const { width, height } = container.getBoundingClientRect();
      renderer.setSize(width, height);
      camera.aspect = width / Math.max(height, 1);
      camera.updateProjectionMatrix();
    });
    observer.observe(container);
    let frame = 0;
    const animate = () => { controls.update(); renderer.render(scene, camera); frame = requestAnimationFrame(animate); };
    animate();
    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      link.listeners.delete(kernel);
      controls.dispose();
      material.dispose();
      grid.geometry.dispose();
      (grid.material as THREE.Material).dispose();
      axes.geometry.dispose();
      (axes.material as THREE.Material).dispose();
      renderer.dispose();
      renderer.domElement.remove();
      materialRef.current = undefined;
    };
  }, [geometry, bounds, kernel, reset, link]);
  useEffect(() => { if (materialRef.current) materialRef.current.wireframe = wireframe; }, [wireframe, geometry, reset]);
  return <div className="viewport" ref={host}>{error && <p className="empty-state">{error}</p>}</div>;
}
