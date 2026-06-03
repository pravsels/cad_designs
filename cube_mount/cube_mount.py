"""63.5 mm cube mount."""

from pathlib import Path

from build123d import *

SIDE = 63.5

with BuildPart() as cube:
    Box(SIDE, SIDE, SIDE)

out_dir = Path(__file__).resolve().parent
part = cube.part
export_step(part, out_dir / "cube_mount.step")
export_stl(part, out_dir / "cube_mount.stl")
print("Exported:", out_dir / "cube_mount.step")
print("Exported:", out_dir / "cube_mount.stl")
print("Bounding box:", part.bounding_box())
