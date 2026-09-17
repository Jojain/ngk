"""Too Tall Toby Party Pack 01-08 Tie Plate, rebuilt with ngk.

The published model is `ppp0108` in
`tests/exchange/foreign/generate/generate_ttt.py`: a steel plate with two
bolted gusset webs, published at 3387.06 g for 7800 kg/m^3 (434238.27 mm^3).

ngk has no sketch/extrude stack, so each piece is built from the primitives the
Python bindings expose:

* the plate is one extrusion of a closed profile made of four lines and the
  four arcs of its rounded slot ends, then five holes are cut through it;
* each web is a pentagon extruded across the plate's thickness, intersected with
  an L-shaped thickness prism, then its bolt hole is cut;
* the three are fused into one solid.

Each piece reproduces the build123d geometry. The web measures 30560.0 mm^3,
which is the oracle's value, and the plate outline is built from the same
tangency points.

The part is shown rather than written: ``write_step`` refuses a solid carrying a
Boolean-cut cylindrical hole ("needs a seam its support has no analytic curve
for"), so this assembly cannot be written to STEP. It still displays.
"""

import math

import ngk

# --------------------------------------------------------------------------- #
# Model constants, from the build123d source.
# --------------------------------------------------------------------------- #

DENSITY_STEEL = 7800 / 1e6  # g/mm^3
PUBLISHED_MASS = 3387.06  # g
PLATE_THICKNESS = 16.0

CAP_RADIUS = 33.0  # radius of a SlotOverall end
CAP_CENTER_X = 188 / 2 - 33  # 61.0
CAP_CENTER_Y = (190 - 2 * 33) / 2  # 62.0
TAB_HALF_HEIGHT = 162 / 2  # 81.0

WEB_TOP = 11.0
WEB_BOTTOM = -19.0
WEB_OUTER_X = 222 / 2 + 14  # 125.0
WEB_STEP_X = WEB_OUTER_X - 40  # 85.0
WEB_INNER_X = WEB_STEP_X - 40  # 45.0
WEB_THIN_HALF = 8 / 2  # 4.0
WEB_THICK_HALF = 20 / 2  # 10.0
WEB_BOLT_X = 222 / 2  # 111.0
WEB_BOLT_Z = -35 + 16 + 14  # -5.0
WEB_BOLT_RADIUS = 11 / 2  # 5.5

# The plate profile's top edge meets each end cap where y = TAB_HALF_HEIGHT.
_CAP_DY = TAB_HALF_HEIGHT - CAP_CENTER_Y
_CAP_DX = math.sqrt(CAP_RADIUS * CAP_RADIUS - _CAP_DY * _CAP_DY)
_CAP_ANGLE = math.atan2(_CAP_DY, -_CAP_DX)


# --------------------------------------------------------------------------- #
# Small helpers over the binding surface.
# --------------------------------------------------------------------------- #


def _frame(origin, x_dir=(1.0, 0.0, 0.0), y_dir=(0.0, 1.0, 0.0)):
    return ngk.Frame.from_xy(ngk.Point(*origin), ngk.Vector(*x_dir), ngk.Vector(*y_dir))


def _box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return ngk.block(x1 - x0, y1 - y0, z1 - z0, frame=_frame((x0, y0, z0)))


def _z_cylinder(cx, cy, z0, z1, radius):
    z0, z1 = sorted((z0, z1))
    return ngk.cylinder(radius, z1 - z0, frame=_frame((cx, cy, z0)))


def _y_cylinder(x, z, y0, y1, radius):
    """A cylinder whose axis runs along +Y, so it bores across the plate."""
    y0, y1 = sorted((y0, y1))
    return ngk.cylinder(
        radius, y1 - y0, frame=_frame((x, y0, z), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    )


def _fuse_all(*solids):
    result = solids[0]
    for solid in solids[1:]:
        result = ngk.fuse(result, solid)
    return result


def _cap_plane(cx, cy):
    return ngk.Plane(ngk.Point(cx, cy, 0.0), ngk.Vector(1, 0, 0), ngk.Vector(0, 0, 1))


# --------------------------------------------------------------------------- #
# The base plate: a rounded-slot outline extruded, then pierced.
# --------------------------------------------------------------------------- #


def base_plate():
    """The 16 mm plate, with its two rounded slot ends and five holes."""
    x_in = CAP_CENTER_X - _CAP_DX
    p0 = (CAP_CENTER_X + CAP_RADIUS, CAP_CENTER_Y, 0.0)
    p1 = (x_in, CAP_CENTER_Y + _CAP_DY, 0.0)
    p2 = (-x_in, CAP_CENTER_Y + _CAP_DY, 0.0)
    p3 = (-(CAP_CENTER_X + CAP_RADIUS), CAP_CENTER_Y, 0.0)
    p4 = (-(CAP_CENTER_X + CAP_RADIUS), -CAP_CENTER_Y, 0.0)
    p5 = (-x_in, -(CAP_CENTER_Y + _CAP_DY), 0.0)
    p6 = (x_in, -(CAP_CENTER_Y + _CAP_DY), 0.0)
    p7 = (CAP_CENTER_X + CAP_RADIUS, -CAP_CENTER_Y, 0.0)

    edges = [
        ngk.line(p7, p0),
        ngk.arc_edge(_cap_plane(CAP_CENTER_X, CAP_CENTER_Y), CAP_RADIUS, 0.0, _CAP_ANGLE),
        ngk.line(p1, p2),
        ngk.arc_edge(
            _cap_plane(-CAP_CENTER_X, CAP_CENTER_Y),
            CAP_RADIUS,
            math.pi - _CAP_ANGLE,
            math.pi,
        ),
        ngk.line(p3, p4),
        ngk.arc_edge(
            _cap_plane(-CAP_CENTER_X, -CAP_CENTER_Y),
            CAP_RADIUS,
            math.pi,
            math.pi + _CAP_ANGLE,
        ),
        ngk.line(p5, p6),
        ngk.arc_edge(
            _cap_plane(CAP_CENTER_X, -CAP_CENTER_Y),
            CAP_RADIUS,
            2 * math.pi - _CAP_ANGLE,
            2 * math.pi,
        ),
    ]
    outline = ngk.modeling.faces.from_profile(ngk.modeling.profiles.from_edges(edges))
    plate = ngk.extrude(outline, ngk.Vector(0, 0, 1), PLATE_THICKNESS)

    for cx, cy, radius in [
        (CAP_CENTER_X, CAP_CENTER_Y, 29 / 2),
        (CAP_CENTER_X, -CAP_CENTER_Y, 29 / 2),
        (-CAP_CENTER_X, CAP_CENTER_Y, 29 / 2),
        (-CAP_CENTER_X, -CAP_CENTER_Y, 29 / 2),
        (0.0, 0.0, 84 / 2),
    ]:
        plate = ngk.cut(plate, _z_cylinder(cx, cy, -1.0, PLATE_THICKNESS + 1.0, radius))
    return plate


# --------------------------------------------------------------------------- #
# A gusset web: the pentagon, thinned to a step, pierced.
# --------------------------------------------------------------------------- #


def side_web(sign):
    """One web, on the `sign` side of YZ.

    The pentagon is built in the XZ plane and extruded across Y. The negative
    side is built as a 180 degree rotation of the positive one rather than a
    mirror, so its shell keeps the same orientation.
    """
    outline = [
        (WEB_INNER_X, 0.0),
        (WEB_INNER_X, WEB_TOP),
        (WEB_OUTER_X, WEB_TOP),
        (WEB_OUTER_X, WEB_BOTTOM),
        (WEB_STEP_X, WEB_BOTTOM),
    ]
    if sign > 0:
        profile = [(x, -WEB_THICK_HALF, z) for x, z in outline]
        direction = ngk.Vector(0, 1, 0)
    else:
        profile = [(-x, WEB_THICK_HALF, z) for x, z in outline]
        direction = ngk.Vector(0, -1, 0)

    web = ngk.extrude(ngk.polygon_face(profile), direction, 2 * WEB_THICK_HALF)

    # The web is 8 mm thick up to the step, 20 mm beyond it. The boxes overlap
    # the web's own faces so the intersection only trims, never extends.
    thickness = _fuse_all(
        _box(sign * (WEB_INNER_X - 5), sign * (WEB_OUTER_X + 5), -WEB_THIN_HALF, WEB_THIN_HALF, -30, 30),
        _box(sign * WEB_STEP_X, sign * (WEB_OUTER_X + 5), -WEB_THICK_HALF, WEB_THICK_HALF, -30, 30),
    )
    web = ngk.intersect(web, thickness)

    return ngk.cut(
        web,
        _y_cylinder(sign * WEB_BOLT_X, WEB_BOLT_Z, -20.0, 20.0, WEB_BOLT_RADIUS),
    )


# --------------------------------------------------------------------------- #
# Assembly.
# --------------------------------------------------------------------------- #


def _show(solid, name):
    """Sends a shape to ocp_vscode, reporting rather than raising with no viewer."""
    try:
        ngk.debug.show(solid, name=name)
    except Exception as error:  # noqa: BLE001 - a missing viewer is not a model error
        print(f"  (could not reach the viewer for {name}: {error})")


def main():
    plate = base_plate()
    right = side_web(+1)
    left = side_web(-1)

    print(f"plate: {plate.face_count} faces")
    print(f"web +X: {right.face_count} faces")
    print(f"web -X: {left.face_count} faces")

    part = _fuse_all(plate, right, left)
    print(f"\nassembled: {part.face_count} faces")
    _show(part, "tie_plate")


if __name__ == "__main__":
    main()
