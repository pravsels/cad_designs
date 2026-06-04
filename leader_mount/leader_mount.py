"""112 x 138 x 74 mm leader mount."""

from pathlib import Path

from build123d import *

WIDTH = 112
DEPTH = 138
HEIGHT = 74

with BuildPart() as leader:
    Box(WIDTH, DEPTH, HEIGHT)

out_dir = Path(__file__).resolve().parent
part = leader.part
export_step(part, out_dir / "leader_mount.step")
export_stl(part, out_dir / "leader_mount.stl")
print("Exported:", out_dir / "leader_mount.step")
print("Exported:", out_dir / "leader_mount.stl")
print("Bounding box:", part.bounding_box())
