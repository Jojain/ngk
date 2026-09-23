import { useCallback, useEffect, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import type { OrthographicCamera, PerspectiveCamera } from "three";
import { CadControls as CadControlsImpl } from "./CadControls";

export type CadControlsViewProps = {
  /** Whether the controls react to user input. */
  enabled?: boolean;
  /** Registers these controls as the canvas default (used by `GizmoHelper`). */
  makeDefault?: boolean;
  /** Trackball center the camera orbits and focuses around. */
  target?: [number, number, number];
  /** Shows the three rotation-circle gizmos around the target. */
  showGizmos?: boolean;
  enablePan?: boolean;
  enableRotate?: boolean;
  enableZoom?: boolean;
  enableFocus?: boolean;
  enableAnimations?: boolean;
  enableGrid?: boolean;
  cursorZoom?: boolean;
  adjustNearFar?: boolean;
  scaleFactor?: number;
  dampingFactor?: number;
  wMax?: number;
  rotateSpeed?: number;
  zoomSpeed?: number;
  panSpeed?: number;
  minDistance?: number;
  maxDistance?: number;
  minZoom?: number;
  maxZoom?: number;
  minFov?: number;
  maxFov?: number;
  radiusFactor?: number;
  focusAnimationTime?: number;
  onChange?: (event: unknown) => void;
  onStart?: (event: unknown) => void;
  onEnd?: (event: unknown) => void;
};

/**
 * React wrapper around the `CadControls` trackball controls. Creates one
 * controls instance bound to the canvas, mirrors the props onto it and drives
 * `update()` every frame. Returns `null`: the controls live in the three scene,
 * not in the React tree.
 */
export default function CadControlsView({
  enabled,
  makeDefault = false,
  target,
  showGizmos = true,
  enablePan,
  enableRotate,
  enableZoom,
  enableFocus,
  enableAnimations,
  enableGrid,
  cursorZoom,
  adjustNearFar,
  scaleFactor,
  dampingFactor,
  wMax,
  rotateSpeed,
  zoomSpeed,
  panSpeed,
  minDistance,
  maxDistance,
  minZoom,
  maxZoom,
  minFov,
  maxFov,
  radiusFactor,
  focusAnimationTime,
  onChange,
  onStart,
  onEnd,
}: CadControlsViewProps) {
  const camera = useThree((s) => s.camera);
  const gl = useThree((s) => s.gl);
  const scene = useThree((s) => s.scene);
  const invalidate = useThree((s) => s.invalidate);
  const set = useThree((s) => s.set);

  const controlsRef = useRef<CadControlsImpl | null>(null);
  const callbacks = useRef({ onChange, onStart, onEnd });
  callbacks.current = { onChange, onStart, onEnd };

  const propsRef = useRef<CadControlsViewProps>({});
  propsRef.current = {
    enabled,
    showGizmos,
    enablePan,
    enableRotate,
    enableZoom,
    enableFocus,
    enableAnimations,
    enableGrid,
    cursorZoom,
    adjustNearFar,
    scaleFactor,
    dampingFactor,
    wMax,
    rotateSpeed,
    zoomSpeed,
    panSpeed,
    minDistance,
    maxDistance,
    minZoom,
    maxZoom,
    minFov,
    maxFov,
    radiusFactor,
    focusAnimationTime,
    target,
  };

  const applyProps = useCallback((controls: CadControlsImpl) => {
    const p = propsRef.current;
    if (p.enabled !== undefined) controls.enabled = p.enabled;
    if (p.enablePan !== undefined) controls.enablePan = p.enablePan;
    if (p.enableRotate !== undefined) controls.enableRotate = p.enableRotate;
    if (p.enableZoom !== undefined) controls.enableZoom = p.enableZoom;
    if (p.enableFocus !== undefined) controls.enableFocus = p.enableFocus;
    if (p.enableAnimations !== undefined)
      controls.enableAnimations = p.enableAnimations;
    if (p.enableGrid !== undefined) controls.enableGrid = p.enableGrid;
    if (p.cursorZoom !== undefined) controls.cursorZoom = p.cursorZoom;
    if (p.adjustNearFar !== undefined) controls.adjustNearFar = p.adjustNearFar;
    if (p.scaleFactor !== undefined) controls.scaleFactor = p.scaleFactor;
    if (p.dampingFactor !== undefined) controls.dampingFactor = p.dampingFactor;
    if (p.wMax !== undefined) controls.wMax = p.wMax;
    if (p.rotateSpeed !== undefined) controls.rotateSpeed = p.rotateSpeed;
    if (p.zoomSpeed !== undefined) controls.zoomSpeed = p.zoomSpeed;
    if (p.panSpeed !== undefined) controls.panSpeed = p.panSpeed;
    if (p.minDistance !== undefined) controls.minDistance = p.minDistance;
    if (p.maxDistance !== undefined) controls.maxDistance = p.maxDistance;
    if (p.minZoom !== undefined) controls.minZoom = p.minZoom;
    if (p.maxZoom !== undefined) controls.maxZoom = p.maxZoom;
    if (p.minFov !== undefined) controls.minFov = p.minFov;
    if (p.maxFov !== undefined) controls.maxFov = p.maxFov;
    if (p.radiusFactor !== undefined) controls.radiusFactor = p.radiusFactor;
    if (p.focusAnimationTime !== undefined)
      controls.focusAnimationTime = p.focusAnimationTime;
    if (p.target) controls.target.set(p.target[0], p.target[1], p.target[2]);
    controls.setGizmosVisible(p.showGizmos ?? true);
    controls.update();
  }, []);

  useEffect(() => {
    const controls = new CadControlsImpl(
      camera as PerspectiveCamera | OrthographicCamera,
      gl.domElement,
      scene,
    );
    controlsRef.current = controls;

    const handleChange = (event: unknown) => {
      invalidate();
      callbacks.current.onChange?.(event);
    };
    const handleStart = (event: unknown) => callbacks.current.onStart?.(event);
    const handleEnd = (event: unknown) => callbacks.current.onEnd?.(event);

    controls.addEventListener("change", handleChange);
    controls.addEventListener("start", handleStart);
    controls.addEventListener("end", handleEnd);

    if (makeDefault) set({ controls });
    applyProps(controls);
    invalidate();

    return () => {
      controls.removeEventListener("change", handleChange);
      controls.removeEventListener("start", handleStart);
      controls.removeEventListener("end", handleEnd);
      if (makeDefault) set({ controls: null });
      controls.dispose();
      controlsRef.current = null;
    };
  }, [camera, gl, scene, makeDefault, set, invalidate, applyProps]);

  const targetX = target?.[0];
  const targetY = target?.[1];
  const targetZ = target?.[2];

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) return;
    applyProps(controls);
    invalidate();
  }, [
    applyProps,
    invalidate,
    enabled,
    showGizmos,
    enablePan,
    enableRotate,
    enableZoom,
    enableFocus,
    enableAnimations,
    enableGrid,
    cursorZoom,
    adjustNearFar,
    scaleFactor,
    dampingFactor,
    wMax,
    rotateSpeed,
    zoomSpeed,
    panSpeed,
    minDistance,
    maxDistance,
    minZoom,
    maxZoom,
    minFov,
    maxFov,
    radiusFactor,
    focusAnimationTime,
    targetX,
    targetY,
    targetZ,
  ]);

  useFrame(() => {
    controlsRef.current?.update();
  });

  return null;
}
