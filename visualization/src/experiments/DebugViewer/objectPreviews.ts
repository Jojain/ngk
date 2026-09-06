type Coordinate3 = {
  x: number;
  y: number;
  z: number;
};

type Coordinate2 = ArrayLike<number>;

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
  Line: (value) => {
    const line = value as { start: Coordinate3; end: Coordinate3 };
    return (
      "Line(" +
      coordinatePreview("P", line.start) +
      " → " +
      coordinatePreview("P", line.end) +
      ")"
    );
  },
  Line2: (value) => {
    const line = value as { start: Coordinate2; end: Coordinate2 };
    return (
      "Line2(" +
      coordinatePreview2("P", line.start) +
      " → " +
      coordinatePreview2("P", line.end) +
      ")"
    );
  },
  Circle: (value) => {
    const circle = value as {
      radius: number;
      plane: { origin: Coordinate3 };
    };
    return (
      "Circle(center=" +
      coordinatePreview("P", circle.plane.origin) +
      ", r=" +
      formatScalar(circle.radius) +
      ")"
    );
  },
  Circle2: (value) => {
    const circle = value as {
      center: Coordinate2;
      radius: number;
      sweep: number;
    };
    return (
      "Circle2(center=" +
      coordinatePreview2("P", circle.center) +
      ", r=" +
      formatScalar(circle.radius) +
      ", sweep=" +
      formatScalar(circle.sweep) +
      ")"
    );
  },
  Cylinder: (value) => {
    const cylinder = value as {
      origin: Coordinate3;
      axis: Coordinate3;
      radius: number;
    };
    return (
      "Cylinder(origin=" +
      coordinatePreview("P", cylinder.origin) +
      ", axis=" +
      coordinatePreview("V", cylinder.axis) +
      ", r=" +
      formatScalar(cylinder.radius) +
      ")"
    );
  },
  Sphere: (value) => {
    const sphere = value as {
      origin: Coordinate3;
      axis: Coordinate3;
      radius: number;
    };
    return (
      "Sphere(origin=" +
      coordinatePreview("P", sphere.origin) +
      ", axis=" +
      coordinatePreview("V", sphere.axis) +
      ", r=" +
      formatScalar(sphere.radius) +
      ")"
      );
    },
  Cone: (value) => {
    const cone = value as {
      origin: Coordinate3;
      axis: Coordinate3;
      referenceRadius: number;
      halfAngle: number;
    };
    return (
      "Cone(origin=" +
      coordinatePreview("P", cone.origin) +
      ", axis=" +
      coordinatePreview("V", cone.axis) +
      ", r=" +
      formatScalar(cone.referenceRadius) +
      ", angle=" +
      formatScalar(cone.halfAngle) +
      ")"
    );
  },
  RuledSurface: (value) => {
    const surface = value as { direction: Coordinate3 };
    return "RuledSurface(direction=" + coordinatePreview("V", surface.direction) + ")";
  },
  SurfaceOfRevolution: (value) => {
    const surface = value as { origin: Coordinate3; axis: Coordinate3 };
    return (
      "SurfaceOfRevolution(origin=" +
      coordinatePreview("P", surface.origin) +
      ", axis=" +
      coordinatePreview("V", surface.axis) +
      ")"
    );
  },
  NurbsCurve: (value) => {
    const curve = value as { degree: number; domain: ArrayLike<number> };
    return "NurbsCurve(degree=" + curve.degree + ", domain=" + intervalPreview(curve.domain) + ")";
  },
  NurbsSurface: (value) => {
    const surface = value as {
      degreeU: number;
      degreeV: number;
      domainU: ArrayLike<number>;
      domainV: ArrayLike<number>;
    };
    return (
      "NurbsSurface(degree=" +
      surface.degreeU +
      "×" +
      surface.degreeV +
      ", u=" +
      intervalPreview(surface.domainU) +
      ", v=" +
      intervalPreview(surface.domainV) +
      ")"
    );
  },
  NurbsCurve2: (value) => {
    const curve = value as {
      degree: number;
      domain: ArrayLike<number>;
    };
    return (
      "NurbsCurve2(degree=" +
      curve.degree +
      ", domain=" +
      intervalPreview(curve.domain) +
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

function coordinatePreview2(prefix: string, value: Coordinate2): string {
  return (
    prefix +
    "(" +
    [value[0], value[1]].map(formatScalar).join(",") +
    ")"
  );
}

function intervalPreview(value: ArrayLike<number>): string {
  return "[" + formatScalar(value[0]) + "," + formatScalar(value[1]) + "]";
}

function formatScalar(value: number): string {
  if (!Number.isFinite(value)) return String(value);
  if (Math.abs(value) < 1e-12) return "0";
  return String(Number(value.toPrecision(6)));
}
