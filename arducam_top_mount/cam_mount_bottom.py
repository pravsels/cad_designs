#!/usr/bin/env python3
"""Interface-first reconstruction of SO-ARM100 cam_mount_bottom.

Built from measurements of reference/cam_mount_bottom.stl. This pass focuses on
the linear-joint end that mates to cam_mount_top, while keeping the long body and
far-end frame close enough for visual fit review.

    .venv/bin/python models/cad_designs/arducam_top_mount/cam_mount_bottom.py \
        --out models/cad_designs/arducam_top_mount/cam_mount_bottom.step \
        --stl-out models/cad_designs/arducam_top_mount/cam_mount_bottom.stl
"""

from __future__ import annotations

import argparse
from pathlib import Path

from build123d import (
    Align,
    Axis,
    BuildLine,
    BuildPart,
    BuildSketch,
    Box,
    Cylinder,
    Plane,
    Polyline,
    export_step,
    export_stl,
)
from build123d import (
    extrude,
    make_face,
)

# --- measured parameters (mm) -------------------------------------------------
BBOX_X = 37.40
BBOX_Y = 230.95
BBOX_Z_MIN = -10.00
BBOX_Z_MAX = 83.02

# Long central body measured from dominant planar faces.
BODY_X_MIN = 6.00
BODY_X_MAX = 31.40
BODY_Y_MIN = 8.70
BODY_Y_MAX = 223.10
BODY_Z_MIN = 18.20
BODY_Z_MAX = 54.83
BODY_FLAT_X_MIN = 7.50
BODY_FLAT_X_MAX = 29.90
BODY_SIDE_Z_MIN = 19.70
BODY_SIDE_Z_MAX = 53.33

# Joint end that receives the top mount's linear-joint interface.
JOINT_Y_MIN = 0.00
JOINT_Y_MAX = 7.20
JOINT_Z_MIN = 0.00
JOINT_Z_MAX = 73.02
JOINT_RAIL_Z_MIN = -10.00
JOINT_RAIL_Z_MAX = 83.02
JOINT_RAIL_X_MIN = 6.85
JOINT_RAIL_X_MAX = 30.55

# Far end frame, kept approximate but in the reference coordinate frame.
FAR_Y_MIN = 223.10
FAR_REAR_Y_MAX = 230.20
FAR_FRONT_Y_MIN = 230.20
FAR_Y_MAX = BBOX_Y
FAR_X_MIN = 11.15
FAR_X_MAX = 26.25
FAR_FRONT_X_MIN = 11.90
FAR_FRONT_X_MAX = 25.50
FAR_Z_MIN = BODY_Z_MIN
FAR_Z_MAX = BODY_Z_MAX
FAR_SLOT_X_MIN = 11.90
FAR_SLOT_X_MAX = 25.50
FAR_SLOT_Y_MIN = 223.10
FAR_SLOT_Y_MAX = 230.95
FAR_SLOT_Z_MIN = 26.05
FAR_SLOT_Z_MAX = 46.98

# Blind side holes along X. Mesh sections show paired blind pockets from each
# side, not one through-cut across the middle of the part.
SIDE_HOLE_R = 1.49 / 2
JOINT_HOLE_Y = 3.60
JOINT_HOLE_ZS = (-4.93, 77.96)
JOINT_HOLE_X_SPANS = ((6.85, 12.95), (24.45, 30.55))
FAR_HOLE_Y = 227.03
FAR_HOLE_ZS = (22.19, 50.83)
FAR_HOLE_X_SPANS = ((11.15, 16.15), (21.25, 26.25))

# Z-axis joint-end bore/boss features reported by stl_inspect. They are useful
# measurement landmarks, but the STL's clipped circular wall is not a simple
# open cylinder. This pass preserves the measured outer envelope; the internal
# round wall can be refined after the mating geometry is accepted.
BIG_BORE_X = 5.97
BIG_BORE_Y = 3.60
BIG_BORE_R = 13.98 / 2
BIG_BORE_Z_MIN = -5.30
BIG_BORE_Z_MAX = 78.32


def _box_from_bounds(x0, x1, y0, y1, z0, z1):
    box = Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))
    return box.translate((x0, y0, z0))


def _xz_prism(points, y0, y1):
    with BuildPart() as part:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(*[(float(x), float(z)) for x, z in points], close=True)
            make_face()
        extrude(amount=y1 - y0)
    solid = part.part
    return solid.translate((0, y0 - solid.bounding_box().min.Y, 0))


def _body_prism(y0, y1):
    # The long bar is a chamfered extrusion, not a rectangular block. These
    # flats are measured from the dominant side/top/bottom planar face extents.
    return _xz_prism(
        [
            (BODY_FLAT_X_MIN, BODY_Z_MIN),
            (BODY_FLAT_X_MAX, BODY_Z_MIN),
            (BODY_X_MAX, BODY_SIDE_Z_MIN),
            (BODY_X_MAX, BODY_SIDE_Z_MAX),
            (BODY_FLAT_X_MAX, BODY_Z_MAX),
            (BODY_FLAT_X_MIN, BODY_Z_MAX),
            (BODY_X_MIN, BODY_SIDE_Z_MAX),
            (BODY_X_MIN, BODY_SIDE_Z_MIN),
        ],
        y0,
        y1,
    )


def _x_cylinder(y, z, x0, x1, radius):
    cyl = Cylinder(radius, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    cyl = cyl.rotate(Axis.Y, 90)
    return cyl.translate((x0, y, z))


def _z_cylinder(x, y, z0, z1, radius):
    cyl = Cylinder(radius, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return cyl.translate((x, y, z0))


def build():
    # Dominant long body plus measured end frames. The reference mesh has local
    # chamfers/curves; this first pass keeps the mate-critical planes exact.
    solid = _body_prism(BODY_Y_MIN, BODY_Y_MAX)
    solid += _box_from_bounds(0.0, BBOX_X, JOINT_Y_MIN, JOINT_Y_MAX, JOINT_Z_MIN, JOINT_Z_MAX)
    solid += _box_from_bounds(JOINT_RAIL_X_MIN, JOINT_RAIL_X_MAX, JOINT_Y_MIN, JOINT_Y_MAX, JOINT_RAIL_Z_MIN, JOINT_RAIL_Z_MAX)
    solid += _box_from_bounds(FAR_X_MIN, FAR_X_MAX, FAR_Y_MIN, FAR_REAR_Y_MAX, FAR_Z_MIN, FAR_Z_MAX)
    solid += _box_from_bounds(FAR_FRONT_X_MIN, FAR_FRONT_X_MAX, FAR_FRONT_Y_MIN, FAR_Y_MAX, FAR_Z_MIN, FAR_Z_MAX)
    solid -= _box_from_bounds(FAR_SLOT_X_MIN, FAR_SLOT_X_MAX, FAR_SLOT_Y_MIN, FAR_SLOT_Y_MAX, FAR_SLOT_Z_MIN, FAR_SLOT_Z_MAX)

    for z in JOINT_HOLE_ZS:
        for x0, x1 in JOINT_HOLE_X_SPANS:
            solid -= _x_cylinder(JOINT_HOLE_Y, z, x0, x1, SIDE_HOLE_R)
    for z in FAR_HOLE_ZS:
        for x0, x1 in FAR_HOLE_X_SPANS:
            solid -= _x_cylinder(FAR_HOLE_Y, z, x0, x1, SIDE_HOLE_R)
    return solid


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True, help="output STEP path")
    parser.add_argument("--stl-out", type=Path, default=None, help="also write a derived STL for scoring")
    args = parser.parse_args(argv)

    solid = build()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    export_step(solid, str(args.out))
    if args.stl_out is not None:
        export_stl(solid, str(args.stl_out))
    bb = solid.bounding_box()
    print(f"cam_mount_bottom pass1: volume {solid.volume:.1f} mm^3")
    print(f"  bbox {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm")
    print(f"  wrote {args.out}" + (f" , {args.stl_out}" if args.stl_out else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
