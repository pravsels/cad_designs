#!/usr/bin/env python3
"""Parametric reconstruction of SO-ARM100 cam_mount_top (webcam overhead mount).

Built from measurements of reference/cam_mount_top.stl (see arducam_top_mount
README). The part is a solid filleted bar extruded along Y with localized
features at each end. This file is the editable source we will later adapt for
the Arducam module (replacing the webcam round joint).

Reconstruction is incremental; each block is measured, not guessed:
  Pass 1 (this file): body bar + the X linear-joint screw holes (the mate).
  Later: far-end holes; replace round joint with Arducam interface.

    .venv/bin/python models/cad_designs/arducam_top_mount/cam_mount_top.py \
        --out /tmp/cmt_param.step --stl-out /tmp/cmt_param.stl
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from build123d import (
    BuildLine,
    BuildPart,
    BuildSketch,
    Plane,
    Polyline,
    RectangleRounded,
    Box,
    export_step,
    export_stl,
    extrude,
    loft,
    make_face,
)

# --- measured parameters (mm) -------------------------------------------------
BODY_X = 25.40        # width  (bbox X)
BODY_Z = 36.62        # height (bbox Z)
BODY_FILLET = 1.5     # corner radius of the bar cross-section
BODY_LEN = 242.45     # length along Y

# Linear-joint screw holes: axis X, near the joint end, one in each side leg.
# They are TEARDROP (self-supporting print) holes whose point aims inward toward
# the centerline (z=0), countersunk at both faces. Measured: neck circle ~dia
# 2.14 opening to ~3.78 at the face over ~3 mm; teardrop apex ~1.4*r past circle.
SCREW_Y = 4.0
SCREW_Z = 14.15          # +/- pair (point aims toward center)
NECK_R = 1.07            # M2 clearance circle radius (neck)
CSK_R = 1.89             # countersink mouth radius at each face
TD_K = 1.4               # teardrop apex distance = TD_K * r from circle center
CSK_DEPTH = 3.0          # face inward to where the neck starts
HALF_X = BODY_X / 2

# Joint-end H-channel: two pockets carved from the +/-Z (top/bottom) faces leave
# two full-height side legs joined by a horizontal central web. The side legs
# carry the countersunk screw holes. (Verified by point-in-solid tests.)
WEB_HALF_X = 7.70        # web spans x = +/-7.7 (gap between the legs)
WEB_HALF_Z = 10.31       # web height: z = +/-10.31 (pocket starts above/below)
CHANNEL_Y = 8.0          # pockets run y=0 -> 8 (back wall at y=8)


def _teardrop_pts(r, cy, cz, point_sign, n=22):
    """Teardrop profile (y,z): circle radius r + apex; apex aims +Z, flipped if
    point_sign<0 so it can point inward toward the centerline."""
    angs = np.radians(np.linspace(45, -225, n))   # circle minus the top wedge
    pts = [(float(cy + r * np.cos(a)), float(cz + r * np.sin(a))) for a in angs]
    pts.append((float(cy), float(cz + TD_K * r)))  # apex (up)
    if point_sign < 0:
        pts = [(y, float(2 * cz - z)) for (y, z) in pts]  # flip apex to -Z
    return pts


def _td_drill(cz):
    """Teardrop through-hole (point toward center) countersunk at both X faces."""
    ps = -1 if cz > 0 else 1                  # apex toward centerline
    xn = HALF_X - CSK_DEPTH
    over = HALF_X + 0.5
    neck = _teardrop_pts(NECK_R, SCREW_Y, cz, ps)
    mouth = _teardrop_pts(CSK_R, SCREW_Y, cz, ps)
    with BuildPart() as d:
        # neck teardrop, extruded through the full width (both legs)
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline(*neck, close=True)
            make_face()
        extrude(amount=over, both=True)
        # +X face countersink: neck -> mouth
        with BuildSketch(Plane.YZ.offset(xn)):
            with BuildLine():
                Polyline(*neck, close=True)
            make_face()
        with BuildSketch(Plane.YZ.offset(over)):
            with BuildLine():
                Polyline(*mouth, close=True)
            make_face()
        loft()
        # -X face countersink
        with BuildSketch(Plane.YZ.offset(-xn)):
            with BuildLine():
                Polyline(*neck, close=True)
            make_face()
        with BuildSketch(Plane.YZ.offset(-over)):
            with BuildLine():
                Polyline(*mouth, close=True)
            make_face()
        loft()
    return d.part


def build():
    # Solid filleted bar: sketch the XZ cross-section, extrude along Y.
    with BuildPart() as part:
        with BuildSketch(Plane.XZ):
            RectangleRounded(BODY_X, BODY_Z, BODY_FILLET)
        extrude(amount=BODY_LEN)
    solid = part.part
    # Position the bar to span Y in [0, BODY_LEN] like the reference frame.
    solid = solid.translate((0, -solid.bounding_box().min.Y, 0))
    # Joint-end H-channel: carve a pocket from the top and bottom (+/-Z) faces
    # (x=+/-WEB_HALF_X, y=0..CHANNEL_Y), leaving two side legs + horizontal web.
    pkt_x = 2 * WEB_HALF_X                        # gap between the two legs
    pkt_y = CHANNEL_Y + 1.0                       # overshoot past the y=0 end
    pkt_z = (BODY_Z / 2 + 1.0) - WEB_HALF_Z       # web top inward to past the face
    for sz in (1, -1):
        pocket = Box(pkt_x, pkt_y, pkt_z)
        pocket = pocket.translate((0, (CHANNEL_Y - 1.0) / 2, sz * (WEB_HALF_Z + pkt_z / 2)))
        solid = solid - pocket
    # Teardrop M2 linear-joint screw holes through the legs (point toward center).
    for z in (SCREW_Z, -SCREW_Z):
        solid = solid - _td_drill(z)
    return solid


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True, help="output STEP path")
    parser.add_argument("--stl-out", type=Path, default=None, help="also write a derived STL (for scoring)")
    args = parser.parse_args(argv)

    solid = build()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    export_step(solid, str(args.out))
    if args.stl_out is not None:
        export_stl(solid, str(args.stl_out))
    print(f"cam_mount_top pass1: volume {solid.volume:.1f} mm^3")
    print(f"  wrote {args.out}" + (f" , {args.stl_out}" if args.stl_out else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
