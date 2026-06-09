"""63.5 mm cube mount with Arducam B0283 pocket on top face.

Takes the outer perimeter of the B0283 board bottom face, offsets it for
FDM clearance, extrudes 2 mm, and boolean-subtracts from the cube top.
"""

from pathlib import Path

from build123d import *
from OCP.gp import gp_GTrsf, gp_Mat
from OCP.BRepBuilderAPI import BRepBuilderAPI_GTransform

# --- Parameters -----------------------------------------------------------

SIDE = 63.5
PEG_DEPTH = 2.0
POCKET_SIZE = 42.6  # mm; B0283 footprint is 42.27, so ~0.165 mm/side clearance

# B0283 footprint from reference STEP (slightly non-square)
_PCB_X = 42.2723
_PCB_Z = 42.2705
_SCALE_X = POCKET_SIZE / _PCB_X
_SCALE_Z = POCKET_SIZE / _PCB_Z

# --- Load B0283 and extract bottom outline --------------------------------

ref_dir = Path(__file__).resolve().parent
b0283 = import_step(ref_dir / "B0283_NAUO7.STEP")

# Bottom face at Y=-3 (board underside)
bottom_face = [
    f for f in b0283.faces()
    if abs(f.center().Y - (-3.0)) < 0.01
    and f.normal_at(f.center()).Y < -0.9
    and f.area > 500
][0]

# Outer wire is the one spanning the full board footprint
outer_wire = max(bottom_face.wires(), key=lambda w: w.length)

# Make a face from just the outer wire, then extrude into a solid tool
outline_face = Face(outer_wire)
tool = extrude(outline_face, amount=PEG_DEPTH)

# Apply clearance: scale in X and Z (footprint axes)
gt = gp_GTrsf()
gt.SetVectorialPart(gp_Mat(
    _SCALE_X, 0, 0,
    0, 1.0, 0,
    0, 0, _SCALE_Z,
))
tool = Solid(BRepBuilderAPI_GTransform(tool.wrapped, gt, True).Shape())

# --- Orient and position on cube top --------------------------------------

# Rotate +90° around X: Y → -Z (board bottom faces down into cube)
tool = tool.rotate(Axis.X, 90)

# Translate so tool top aligns with cube top (Z = SIDE/2)
tool_bb = tool.bounding_box()
tool = tool.translate((0, 0, SIDE / 2 - tool_bb.max.Z))

# --- Mounting pegs (fill the 4 hole cutouts in the pocket) ----------------

# The 4 mounting holes on the B0283 bottom face are 2.7mm diameter circles.
# We add solid pegs at those positions so the board plugs onto them.
# Peg positions are in B0283 native coords (X, Z); after rotation X stays X,
# Z becomes -Y in cube coords — but we placed everything centered, so the
# peg centers in cube XY are (B0283_X, -B0283_Z) after the +90° X rotation.
PEG_POSITIONS_B0283 = [
    (+12.996, +18.135),
    (+12.996, -18.135),
    (-12.996, +18.135),
    (-12.996, -18.135),
]
PEG_OD = 2.7
POCKET_FLOOR_Z = SIDE / 2 - PEG_DEPTH

pegs = []
for bx, bz in PEG_POSITIONS_B0283:
    peg = Cylinder(
        PEG_OD / 2, PEG_DEPTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    peg = peg.locate(Location((bx, -bz, POCKET_FLOOR_Z)))
    pegs.append(peg)

# --- Build ----------------------------------------------------------------

cube = Box(SIDE, SIDE, SIDE)
part = cube - tool
for peg in pegs:
    part = part + peg

# --- Export ---------------------------------------------------------------

out_dir = ref_dir
export_step(part, out_dir / "cube_mount_arducam.step")
export_stl(part, out_dir / "cube_mount_arducam.stl")

bb = part.bounding_box()
print(f"Exported: {out_dir / 'cube_mount_arducam.step'}")
print(f"Exported: {out_dir / 'cube_mount_arducam.stl'}")
print(f"Bounding box: {bb.size.X:.3f} x {bb.size.Y:.3f} x {bb.size.Z:.3f}")
print(f"Pocket depth: {PEG_DEPTH:.1f} mm")
print(f"Pocket opening: {POCKET_SIZE:.1f} x {POCKET_SIZE:.1f} mm")
