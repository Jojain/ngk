from ngk.geometry import Point, Plane, Vector
from ngk.modeling import booleans, faces, loft
from ngk.viz import debug

# Two square-to-circle lofts crossing at right angles, then fused.
#
# Both sections face along the run: a section edge-on to it has no side for the
# cap to face and lofts to a solid that cannot be told inside from outside.
#
# `faces.rectangle` anchors its first corner at the plane origin while
# `faces.circle` centres on it, so each square's corner is placed to put the
# square around the axis its loft runs along.
#
# The two runs are deliberately *not* congruent. Every side of a square-to-
# circle loft is tangent to the plane of the square edge it grew from, along
# that side's own middle. Give the second loft the same section as the first
# and the two solids meet tangentially rather than crossing -- a contact the
# Boolean refuses by name instead of guessing at. A different section size
# puts those tangent planes apart and the sides cross cleanly.

# Runs up the z axis: a 4 x 4 square at z = 0 to a circle of radius 2 at z = 5.
upright = loft.faces(
    [
        faces.rectangle(
            4.0, 4.0, plane=Plane(Point(-2, -2, 0), Vector(1, 0, 0), Vector(0, 0, 1))
        ),
        faces.circle(2.0, plane=Plane(Point(0, 0, 5), Vector(1, 0, 0), Vector(0, 0, 1))),
    ]
)
print(f"Upright loft: {upright.face_count} faces")

# Runs along x, right through the upright one: a 2.5 x 2.5 square at x = -3 to
# a circle of radius 1 at x = 6, both centred on the line y = 0, z = 2.
sideways = loft.faces(
    [
        faces.rectangle(
            2.5, 2.5, plane=Plane(Point(-3, -1.25, 0.75), Vector(0, 1, 0), Vector(1, 0, 0))
        ),
        faces.circle(1.0, plane=Plane(Point(6, 0, 2), Vector(0, 1, 0), Vector(1, 0, 0))),
    ]
)
print(f"Sideways loft: {sideways.face_count} faces")

fused = booleans.fuse(upright, sideways)
print(f"Fused: {fused.face_count} faces")

debug.show(fused)
