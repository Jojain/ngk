import sys
from math import asin, cos, pi, radians, sin, sqrt, tan
from pathlib import Path

import build123d as bd
from build123d import *  # noqa: F403 — the published models are written this way

OUT_DIR = Path(__file__).resolve().parents[1] / "files"

DENSA = 7800 / 1e6  # carbon steel, g/mm^3
DENSB = 2700 / 1e6  # aluminium alloy
DENSC = 1020 / 1e6  # ABS


def ppp0101():
    """Party Pack 01-01 Bearing Bracket."""

    with BuildPart() as p:
        with BuildSketch() as s:
            Rectangle(115, 50)
            with Locations((5 / 2, 0)):
                SlotOverall(90, 12, mode=Mode.SUBTRACT)
        extrude(amount=15)

        with BuildSketch(Plane.XZ.offset(50 / 2)) as s3:
            with Locations((-115 / 2 + 26, 15)):
                SlotOverall(42 + 2 * 26 + 12, 2 * 26, rotation=90)
        zz = extrude(amount=-12)
        split(bisect_by=Plane.XY)
        edgs = p.part.edges().filter_by(Axis.Y).group_by(Axis.X)[-2]
        fillet(edgs, 9)

        with Locations(zz.faces().sort_by(Axis.Y)[0]):
            with Locations((42 / 2 + 6, 0)):
                CounterBoreHole(24 / 2, 34 / 2, 4)
        mirror(about=Plane.XZ)

        with BuildSketch() as s4:
            RectangleRounded(115, 50, 6)
        extrude(amount=80, mode=Mode.INTERSECT)

        with BuildSketch(Plane.YZ) as s4:
            with BuildLine() as bl:
                l1 = Line((0, 0), (18 / 2, 0))
                l2 = PolarLine(l1 @ 1, 8, 60, length_mode=LengthMode.VERTICAL)
                l3 = Line(l2 @ 1, (0, 8))
                mirror(about=Plane.YZ)
            make_face()
        extrude(amount=115 / 2, both=True, mode=Mode.SUBTRACT)

    return p.part, p.part.volume * DENSA, 797.15


def ppp0102():
    """Party Pack 01-02 Post Cap."""

    with BuildPart() as p:
        with BuildSketch(Plane.XZ) as sk1:
            Rectangle(49, 48 - 8, align=(Align.CENTER, Align.MIN))
            Rectangle(9, 48, align=(Align.CENTER, Align.MIN))
            with Locations((9 / 2, 40)):
                Ellipse(20, 8)
            split(bisect_by=Plane.YZ)
        revolve(axis=Axis.Z)

        with BuildSketch(Plane.YZ.offset(-15)) as xc1:
            with Locations((0, 40 / 2 - 17)):
                Ellipse(10 / 2, 4 / 2)
            with BuildLine(Plane.XZ) as l1:
                CenterArc((-15, 40 / 2), 17, 90, 180)
        sweep(path=l1)

        fillet(p.edges().filter_by(GeomType.CIRCLE, reverse=True).group_by(Axis.X)[0], 1)

        with BuildLine(mode=Mode.PRIVATE) as lc1:
            PolarLine((42 / 2, 0), 37, 94, length_mode=LengthMode.VERTICAL)

        pts = [
            (0, 0),
            (42 / 2, 0),
            ((lc1.line @ 1).X, (lc1.line @ 1).Y),
            (0, (lc1.line @ 1).Y),
        ]
        with BuildSketch(Plane.XZ) as sk2:
            Polygon(*pts, align=None)
            fillet(sk2.vertices().group_by(Axis.X)[1], 3)
        revolve(axis=Axis.Z, mode=Mode.SUBTRACT)

    return p.part, p.part.volume * DENSC, 43.09


def ppp0103():
    """Party Pack 01-03 C Clamp Base."""

    with BuildPart() as ppp0103:
        with BuildSketch() as sk1:
            RectangleRounded(34 * 2, 95, 18)
            with Locations((0, -2)):
                RectangleRounded((34 - 16) * 2, 95 - 18 - 14, 7, mode=Mode.SUBTRACT)
            with Locations((-34 / 2, 0)):
                Rectangle(34, 95, 0, mode=Mode.SUBTRACT)
        extrude(amount=16)
        with BuildSketch(Plane.XZ.offset(-95 / 2)) as cyl1:
            with Locations((0, 16 / 2)):
                Circle(16 / 2)
        extrude(amount=18)
        with BuildSketch(Plane.XZ.offset(95 / 2 - 14)) as cyl2:
            with Locations((0, 16 / 2)):
                Circle(16 / 2)
        extrude(amount=23)
        with Locations(Plane.XZ.offset(95 / 2 + 9)):
            with Locations((0, 16 / 2)):
                CounterSinkHole(5.5 / 2, 11.2 / 2, None, 90)

    return ppp0103.part, ppp0103.part.volume * DENSB, 96.13


def ppp0104():
    """Party Pack 01-04 Angle Bracket."""

    d1, d2, d3 = 38, 26, 16
    h1, h2, h3, h4 = 20, 8, 7, 23
    w1, w2, w3 = 80, 10, 5
    f1, f2, f3 = 4, 10, 5
    sloth1, sloth2 = 18, 12
    slotw1, slotw2 = 17, 14

    with BuildPart() as p:
        with BuildSketch() as s:
            Circle(d1 / 2)
        extrude(amount=h1)
        with BuildSketch(Plane.XY.offset(h1)) as s2:
            Circle(d2 / 2)
        extrude(amount=h2)
        with BuildSketch(Plane.YZ) as s3:
            Rectangle(d1 + 15, h3, align=(Align.CENTER, Align.MIN))
        extrude(amount=w1 - d1 / 2)
        ped = p.part.edges().group_by(Axis.Z)[2].filter_by(GeomType.CIRCLE)
        fillet(ped, f1)
        with BuildSketch(Plane.YZ) as s3a:
            Rectangle(d1 + 15, 15, align=(Align.CENTER, Align.MIN))
            Rectangle(d1, 15, mode=Mode.SUBTRACT, align=(Align.CENTER, Align.MIN))
        extrude(amount=w1 - d1 / 2, mode=Mode.SUBTRACT)
        with BuildSketch() as s4:
            Circle(d3 / 2)
        extrude(amount=h1 + h2, mode=Mode.SUBTRACT)
        with BuildSketch() as s5:
            with Locations((w1 - d1 / 2 - w2 / 2, 0)):
                Rectangle(w2, d1)
        extrude(amount=-h4)
        fillet(p.part.edges().group_by(Axis.X)[-1].sort_by(Axis.Z)[-1], f2)
        fillet(p.part.edges().group_by(Axis.X)[-4].sort_by(Axis.Z)[-2], f3)
        pln = Plane.YZ.offset(w1 - d1 / 2)
        with BuildSketch(pln) as s6:
            with Locations((0, -h4)):
                SlotOverall(slotw1 * 2, sloth1, 90)
        extrude(amount=-w3, mode=Mode.SUBTRACT)
        with BuildSketch(pln) as s6b:
            with Locations((0, -h4)):
                SlotOverall(slotw2 * 2, sloth2, 90)
        extrude(amount=-w2, mode=Mode.SUBTRACT)

    return p.part, p.part.volume * DENSA, 310


def ppp0105():
    """Party Pack 01-05 Paste Sleeve."""

    with BuildPart() as p:
        with BuildSketch() as s:
            SlotOverall(45, 38)
            offset(amount=3)
        with BuildSketch(Plane.XY.offset(133 - 30)) as s2:
            SlotOverall(60, 4)
            offset(amount=3)
        loft()

        with BuildSketch() as s3:
            SlotOverall(45, 38)
        with BuildSketch(Plane.XY.offset(133 - 30)) as s4:
            SlotOverall(60, 4)
        loft(mode=Mode.SUBTRACT)

        extrude(p.part.faces().sort_by(Axis.Z)[0], amount=30)

    return p.part, p.part.volume * DENSC, 57.08


def ppp0106():
    """Party Pack 01-06 Bearing Jig."""

    r1, r2, r3, r4, r5 = 30 / 2, 13 / 2, 12 / 2, 10, 6
    x1 = 44
    y1, y2, y3, y4, y_tot = 36, 36 - 22 / 2, 22 / 2, 42, 69

    with BuildSketch(Location((0, -r1, y3))) as sk_body:
        with BuildLine() as l:
            c1 = Line((r1, 0), (r1, y_tot), mode=Mode.PRIVATE)
            m1 = Line((0, y_tot), (x1 / 2, y_tot))
            m2 = JernArc(m1 @ 1, m1 % 1, r4, -90 - 45)
            m3 = IntersectingLine(m2 @ 1, m2 % 1, c1)
            m4 = Line(m3 @ 1, (r1, r1))
            m5 = JernArc(m4 @ 1, m4 % 1, r1, -90)
            mirror(about=Plane.YZ)
        make_face()
        fillet(sk_body.vertices().group_by(Axis.Y)[1], 12)
        with Locations((x1 / 2, y_tot - 10), (-x1 / 2, y_tot - 10)):
            Circle(r2, mode=Mode.SUBTRACT)
        with Locations((0, r1)):
            Circle(r3, mode=Mode.SUBTRACT)
            Rectangle(4, 3 + 6, align=(Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    with BuildPart() as p:
        Box(200, 200, 22)
        Cylinder(r1, y2, align=(Align.CENTER, Align.CENTER, Align.MAX))
        fillet(p.edges(Select.NEW), r5)
        extrude(sk_body.sketch, amount=-y1, mode=Mode.INTERSECT)
        with Locations((0, y_tot - r1 - y4, 0)):
            Box(
                y_tot,
                y_tot,
                10,
                align=(Align.CENTER, Align.MIN, Align.CENTER),
                mode=Mode.SUBTRACT,
            )

    return p.part, p.part.volume * DENSA, 328.02


def ppp0107():
    """Party Pack 01-07 Flanged Hub."""

    with BuildPart() as p:
        with BuildSketch() as s:
            Circle(130 / 2)
        extrude(amount=8)
        with BuildSketch(Plane.XY.offset(8)) as s2:
            Circle(84 / 2)
        extrude(amount=25 - 8)
        with BuildSketch(Plane.XY.offset(25)) as s3:
            Circle(35 / 2)
        extrude(amount=52 - 25)
        with BuildSketch() as s4:
            Circle(73 / 2)
        extrude(amount=18, mode=Mode.SUBTRACT)
        pln2 = p.part.faces().sort_by(Axis.Z)[5]
        with BuildSketch(Plane.XY.offset(52)) as s5:
            Circle(20 / 2)
        extrude(amount=-52, mode=Mode.SUBTRACT)
        fillet(
            p.part.edges()
            .filter_by(GeomType.CIRCLE)
            .sort_by(Axis.Z)[2:-2]
            .sort_by(SortBy.RADIUS)[1:],
            3,
        )
        pln = Plane(pln2)
        pln.origin = pln.origin + Vector(20 / 2, 0, 0)
        pln = pln.rotated((0, 45, 0))
        pln = pln.offset(-25 + 3 + 0.10)
        with BuildSketch(pln) as s6:
            Rectangle((73 - 35) / 2 * 1.414 + 5, 3)
        zz = extrude(amount=15, taper=-20 / 2, mode=Mode.PRIVATE)
        zz2 = split(zz, bisect_by=Plane.XY.offset(25), mode=Mode.PRIVATE)
        zz3 = split(zz2, bisect_by=Plane.YZ.offset(35 / 2 - 1), mode=Mode.PRIVATE)
        with PolarLocations(0, 3):
            insert(zz3)
        with Locations(Plane.XY.offset(8)):
            with PolarLocations(107.95 / 2, 6):
                CounterBoreHole(6 / 2, 13 / 2, 4)

    return p.part, p.part.volume * DENSB, 372.99


def ppp0108():
    """Party Pack 01-08 Tie Plate."""

    with BuildPart() as p:
        with BuildSketch() as s1:
            Rectangle(188 / 2 - 33, 162, align=(Align.MIN, Align.CENTER))
            with Locations((188 / 2 - 33, 0)):
                SlotOverall(190, 33 * 2, rotation=90)
            mirror(about=Plane.YZ)
            with GridLocations(188 - 2 * 33, 190 - 2 * 33, 2, 2):
                Circle(29 / 2, mode=Mode.SUBTRACT)
            Circle(84 / 2, mode=Mode.SUBTRACT)
        extrude(amount=16)

        with BuildPart() as p2:
            with BuildSketch(Plane.XZ) as s2:
                with BuildLine() as l1:
                    l1 = Polyline(
                        (222 / 2 + 14 - 40 - 40, 0),
                        (222 / 2 + 14 - 40, -35 + 16),
                        (222 / 2 + 14, -35 + 16),
                        (222 / 2 + 14, -35 + 16 + 30),
                        (222 / 2 + 14 - 40 - 40, -35 + 16 + 30),
                        close=True,
                    )
                make_face()
                with Locations((222 / 2, -35 + 16 + 14)):
                    Circle(11 / 2, mode=Mode.SUBTRACT)
            extrude(amount=20 / 2, both=True)
            with BuildSketch() as s3:
                with Locations(l1 @ 0):
                    Rectangle(40 + 40, 8, align=(Align.MIN, Align.CENTER))
                    with Locations((40, 0)):
                        Rectangle(40, 20, align=(Align.MIN, Align.CENTER))
            extrude(amount=30, both=True, mode=Mode.INTERSECT)
            mirror(about=Plane.YZ)

    return p.part, p.part.volume * DENSA, 3387.06


def ppp0109():
    """Party Pack 01-09 Corner Tie."""


    with BuildPart() as ppp109:
        with BuildSketch() as one:
            Rectangle(69, 75, align=(Align.MAX, Align.CENTER))
            fillet(one.vertices().group_by(Axis.X)[0], 17)
        extrude(amount=13)
        centers = [
            arc.arc_center
            for arc in ppp109.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[-1]
        ]
        with Locations(*centers):
            CounterBoreHole(
                radius=8 / 2, counter_bore_radius=15 / 2, counter_bore_depth=4
            )

        with BuildSketch(Plane.YZ) as two:
            with Locations((0, 45)):
                Circle(15)
            with BuildLine() as bl:
                c = Line((75 / 2, 0), (75 / 2, 60), mode=Mode.PRIVATE)
                u = two.edge().find_tangent(75 / 2 + 90)[0]
                l1 = IntersectingLine(
                    two.edge().position_at(u), -two.edge().tangent_at(u), other=c
                )
                Line(l1 @ 0, (0, 45))
                Polyline((0, 0), c @ 0, l1 @ 1)
                mirror(about=Plane.YZ)
            make_face()
            with Locations((0, 45)):
                Circle(12 / 2, mode=Mode.SUBTRACT)
        extrude(amount=-13)

        with BuildSketch(Plane((0, 0, 0), x_dir=(1, 0, 0), z_dir=(1, 0, 1))) as three:
            Rectangle(45 * 2 / sqrt(2) - 37.5, 75, align=(Align.MIN, Align.CENTER))
            with Locations(three.edges().sort_by(Axis.X)[-1].center()):
                Circle(37.5)
                Circle(33 / 2, mode=Mode.SUBTRACT)
            split(bisect_by=Plane.YZ)
        extrude(amount=6)
        f = ppp109.faces().filter_by(Axis((0, 0, 0), (-1, 0, 1)))[0]
        extrude(f, until=Until.NEXT)
        fillet(ppp109.edges().filter_by(Axis.Y).sort_by(Axis.Z)[2], 16)

    return ppp109.part, ppp109.part.volume * DENSB, 307.23


def ppp0110():
    """Party Pack 01-10 Light Cap."""


    OT = 40
    OP = sqrt((-84 / 2) ** 2 + (-6) ** 2)
    TP = sqrt(OP**2 - 40**2)
    OPT_degrees = asin(OT / OP) * 180 / pi
    OP_to_X_axis_degrees = asin(6 / OP) * 180 / pi
    left_tangent_degrees = OPT_degrees + OP_to_X_axis_degrees
    left_tangent_length = TP

    with BuildPart() as outer:
        with BuildSketch(Plane.XZ) as sk:
            with BuildLine():
                l1 = PolarLine(
                    start=(-84 / 2, 0),
                    length=left_tangent_length,
                    angle=left_tangent_degrees,
                )
                l2 = TangentArc(l1 @ 1, (0, 46), tangent=l1 % 1)
                l3 = offset(amount=-8, side=Side.RIGHT, closed=False, mode=Mode.ADD)
                l4 = Line(l1 @ 0, l3 @ 1)
                l5 = Line(l3 @ 0, l2 @ 1)
            make_face()

            with BuildLine():
                l6 = Line(l2 @ 1, (0, 46 - 16))
                l7 = IntersectingLine(start=l6 @ 1, direction=(-1, 0), other=l3)
                l8 = TangentArc(l7 @ 1, l2 @ 1, tangent=(-1, 0), tangent_from_first=False)

            make_face()

        revolve(axis=Axis.Z)
    sk = sk.sketch & Plane.XZ * Rectangle(
        1000, 1000, align=[Align.CENTER, Align.MIN]
    )
    positive_Z = Box(100, 100, 100, align=[Align.CENTER, Align.MIN, Align.MIN])
    p = outer.part & positive_Z
    cross_section = sk + mirror(sk, about=Plane.YZ)
    p += extrude(cross_section, amount=50)
    p += mirror(p, about=Plane.XZ.offset(50))
    p += fillet(
        p.edges().filter_by(GeomType.LINE).filter_by(Axis.Y).group_by(Axis.Z)[-1],
        radius=8,
    )

    return p, p.volume * DENSC, 211.30


def sm_hanger():
    """23-02-02 SM Hanger."""

    sheet_thickness = 4 * MM

    with BuildPart() as side:
        with BuildLine(Plane.XZ) as side_line:
            l1 = Line((0, 65), (170 / 2, 65))
            l2 = PolarLine(
                l1 @ 1,
                length=65,
                direction=(0.5, -0.866025403784),
                length_mode=LengthMode.VERTICAL,
            )
            l3 = Line(l2 @ 1, (170 / 2, 0))
            fillet(side_line.vertices(), 7)
        make_brake_formed(
            thickness=sheet_thickness,
            station_widths=[40, 40, 40, 112.52 / 2, 112.52 / 2, 112.52 / 2],
            side=Side.RIGHT,
        )
        if side.vertices().sort_by(Axis.X)[0].X < -sheet_thickness:
            mirror(about=Plane.YZ, mode=Mode.REPLACE)
        fe = side.edges().filter_by(Axis.Z).group_by(Axis.Z)[0].sort_by(Axis.Y)[-1]
        fillet(fe, radius=7)

    with BuildPart() as wing:
        with BuildLine(Plane.YZ) as wing_line:
            l1 = Line((0, 65), (80 / 2 + 1.526 * sheet_thickness, 65))
            PolarLine(l1 @ 1, 20.371288916, direction=(0.258819045103, -0.965925826289))
            fillet(wing_line.vertices(), 7)
        make_brake_formed(
            thickness=sheet_thickness,
            station_widths=110 / 2,
            side=Side.RIGHT,
        )
        if wing.vertices().sort_by(Axis.X)[0].X < -sheet_thickness:
            mirror(about=Plane.YZ, mode=Mode.REPLACE)
        bottom_edge = wing.edges().group_by(Axis.X)[-1].sort_by(Axis.Z)[0]
        fillet(bottom_edge, radius=7)

    tab_line = Plane.XZ * Polyline(
        (20, 65 - sheet_thickness), (56 / 2, 65 - sheet_thickness), (56 / 2, 88)
    )
    tab_line = fillet(tab_line.vertices(), 7)
    tab = make_brake_formed(sheet_thickness, 8, tab_line, Side.RIGHT)
    if tab.vertices().sort_by(Axis.Y)[0].Y < -sheet_thickness:
        tab = mirror(tab, about=Plane.XZ)
    tab = fillet(
        tab.edges().filter_by(Axis.X).group_by(Axis.Z)[-1].sort_by(Axis.Y)[-1], 5
    )
    tab -= Pos((0, 0, 80)) * Rot(0, 90, 0) * Hole(5, 100)

    with BuildPart() as hanger:
        insert([side.part, wing.part])
        mirror(about=Plane.XZ)
        with BuildSketch(Plane.XY.offset(65)) as h1:
            with Locations((20, 0)):
                Rectangle(30, 30, align=(Align.MIN, Align.CENTER))
                fillet(h1.vertices().group_by(Axis.X)[-1], 7)
            SlotCenterPoint((154, 0), (154 / 2, 0), 20)
        extrude(amount=-40, mode=Mode.SUBTRACT)
        with BuildSketch() as h2:
            SlotCenterPoint((206, 0), (206 / 2, 0), 20)
        extrude(amount=40, mode=Mode.SUBTRACT)
        insert(tab)
        mirror(about=Plane.YZ)
        mirror(about=Plane.XZ)

    return hanger.part, hanger.part.volume * DENSA, 1028


def curved_support():
    """23-T-24 Curved Support."""

    import sympy

    y30, x66, xl8, yl8 = sympy.symbols("y30 x66 xl8 yl8")
    x30 = 77 - 55 / 2
    y66 = 66 + 32

    equations = [
        (x66 - x30) ** 2 + (y66 - y30) ** 2 - (66 + 30) ** 2,
        xl8 - (x30 + 30 * sin(radians(8))),
        yl8 - (y30 + 30 * cos(radians(8))),
        (yl8 - 50) / (55 / 2 - xl8) - tan(radians(8)),
    ]
    solution = {k: float(v) for k, v in sympy.solve(equations, dict=True)[1].items()}

    c30 = Vector(x30, solution[y30])
    c66 = Vector(solution[x66], y66)
    l8 = Vector(solution[xl8], solution[yl8])
    i30_66 = Line(c30, c66) @ (30 / (30 + 66))
    lh = Vector(c66.X, 32)

    with BuildLine() as profile:
        l1 = Line((55 / 2, 50), l8)
        l2 = RadiusArc(l1 @ 1, i30_66, 30)
        l3 = RadiusArc(l2 @ 1, lh, -66)
        l4 = Polyline(l3 @ 1, (125, 32), (125, 0), (0, 0), (0, (l1 @ 0).Y), l1 @ 0)

    with BuildPart() as support:
        with BuildSketch() as base_plan:
            c_8_degrees = Circle(55 / 2)
            with Locations((0, 125)):
                Circle(30 / 2)
            base_hull = make_hull(mode=Mode.PRIVATE)
        extrude(amount=32)
        extrude(c_8_degrees, amount=60)
        extrude(base_hull, amount=11)
        with BuildSketch(Plane.YZ) as bridge:
            make_face(profile.edges())
        extrude(amount=11 / 2, both=True)
        Hole(35 / 2)
        with Locations((0, 125)):
            Hole(20 / 2)

    return support.part, support.part.volume * DENSA, 1294


def buffer_stand():
    """24-SPO-06 Buffer Stand."""

    with BuildPart() as p:
        with BuildSketch() as xy:
            with BuildLine():
                l1 = ThreePointArc((5 / 2, -1.25), (5.5 / 2, 0), (5 / 2, 1.25))
                Polyline(l1 @ 0, (0, -1.25), (0, 1.25), l1 @ 1)
            make_face()
        extrude(amount=4)

        with BuildSketch(Plane.YZ) as yz:
            Trapezoid(2.5, 4, 90 - 6, align=(Align.CENTER, Align.MIN))
            full_round(yz.edges().sort_by(SortBy.LENGTH)[0])
            circle_edge = yz.edges().filter_by(GeomType.CIRCLE)[0]
            arc_center = circle_edge.arc_center
            arc_radius = circle_edge.radius
        extrude(amount=10, mode=Mode.INTERSECT)

        with BuildPart(mode=Mode.SUBTRACT) as internals:
            y = p.edges().filter_by(Axis.X).sort_by(Axis.Z)[-1].center().Z

            with BuildSketch(Plane.YZ.offset(4.25 / 2)) as yz:
                Trapezoid(2.5, y, 90 - 6, align=(Align.CENTER, Align.MIN))
                with Locations(arc_center):
                    Circle(arc_radius, mode=Mode.SUBTRACT)
            extrude(amount=-(4.25 - 3.5) / 2)

            with BuildSketch(Plane.YZ.offset(3.5 / 2)) as yz:
                Trapezoid(2.5, 4, 90 - 6, align=(Align.CENTER, Align.MIN))
            extrude(amount=-3.5 / 2)

            with BuildSketch(Plane.XZ.offset(-2)) as xz:
                with Locations((0, 4)):
                    RectangleRounded(4.25, 7.5, 0.5)
            extrude(amount=4, mode=Mode.INTERSECT)

        with Locations(
            p.faces(Select.LAST).filter_by(GeomType.PLANE).sort_by(Axis.Z)[-1]
        ):
            CounterBoreHole(0.625 / 2, 1.25 / 2, 0.5)

        with BuildSketch(Plane.YZ) as rib:
            with Locations((0, 0.25)):
                Trapezoid(0.5, 1, 90 - 8, align=(Align.CENTER, Align.MIN))
            full_round(rib.edges().sort_by(SortBy.LENGTH)[0])
        extrude(amount=4.25 / 2)

        mirror(about=Plane.YZ)

    part = scale(p.part, IN)
    return part, part.volume * 7800e-6 / 453.59237, 3.923


MODELS = [
    ("ppp0101_bearing_bracket", ppp0101),
    ("ppp0102_post_cap", ppp0102),
    ("ppp0103_c_clamp_base", ppp0103),
    ("ppp0104_angle_bracket", ppp0104),
    ("ppp0105_paste_sleeve", ppp0105),
    ("ppp0106_bearing_jig", ppp0106),
    ("ppp0107_flanged_hub", ppp0107),
    ("ppp0108_tie_plate", ppp0108),
    ("ppp0109_corner_tie", ppp0109),
    ("ppp0110_light_cap", ppp0110),
    ("sm_hanger", sm_hanger),
    ("curved_support", curved_support),
    ("buffer_stand", buffer_stand),
]


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    built = 0
    for name, build in MODELS:
        try:
            part, got_mass, want_mass = build()
        except Exception as error:  # noqa: BLE001 — report every model, build the rest
            print(f"  {name}: FAILED to build — {type(error).__name__}: {error}")
            continue
        bd.export_step(part, str(OUT_DIR / f"{name}.step"))
        print(f"  {name}: {len(part.faces())} faces (mass {got_mass:.2f}, published {want_mass})")
        built += 1
    print(f"wrote {built} of {len(MODELS)} into {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
