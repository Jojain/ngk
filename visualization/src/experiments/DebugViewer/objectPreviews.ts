type Coordinate3 = {
  x: number;
  y: number;
  z: number;
};

export type ObjectPreviewFormatter = (value: object) => string;

// Register compact, side-effect-free debugger presentations by WASM class name.
export const OBJECT_PREVIEW_FORMATTERS: Record<
  string,
  ObjectPreviewFormatter
> = {
  Point3: (value) => coordinatePreview("P", value as Coordinate3),
  Vector3: (value) => coordinatePreview("V", value as Coordinate3),
  Plane: (value) => {
    const plane = value as {
      origin: Coordinate3;
      normal: Coordinate3;
    };
    return (
      "Plane(origin=" +
      coordinatePreview("P", plane.origin) +
      ", normal=" +
      coordinatePreview("V", plane.normal) +
      ")"
    );
  },
};

export function objectPreview(value: object): string | null {
  const typeName = value.constructor?.name;
  if (!typeName) return null;
  const formatter = OBJECT_PREVIEW_FORMATTERS[typeName];
  if (!formatter) return null;
  try {
    return formatter(value);
  } catch {
    return typeName;
  }
}

function coordinatePreview(prefix: string, value: Coordinate3): string {
  return (
    prefix +
    "(" +
    [value.x, value.y, value.z].map(formatScalar).join(",") +
    ")"
  );
}

function formatScalar(value: number): string {
  if (!Number.isFinite(value)) return String(value);
  if (Math.abs(value) < 1e-12) return "0";
  return String(Number(value.toPrecision(6)));
}
