"""3-layer staircase shelf: back panel with progressively shorter shelf tiles."""

from pathlib import Path

from build123d import *

WIDTH = 180      # mm (x)
DEPTH = 100      # mm (y)
HEIGHT = 100     # mm (z)
THICKNESS = 3    # mm - panel and shelf thickness
NUM_LAYERS = 2

# Shelf z-positions: evenly spaced from bottom to top
shelf_zs = [i * (HEIGHT - THICKNESS) / (NUM_LAYERS - 1) for i in range(NUM_LAYERS)]

# Each shelf is 75% shorter than the one below (staircase)
full_depth = DEPTH - THICKNESS  # net depth excluding backbone
shelf_depths = [full_depth * (0.75 ** i) for i in range(NUM_LAYERS)]

with BuildPart() as shelf:
    # Back panel (backbone): full width, full height, at the rear edge
    back_y = -(DEPTH - THICKNESS) / 2
    with Locations([(0, back_y, HEIGHT / 2)]):
        Box(WIDTH, THICKNESS, HEIGHT)

    # Shelf tiles: each starts flush against backbone, progressively shorter
    for z, sd in zip(shelf_zs, shelf_depths):
        shelf_y = back_y + THICKNESS / 2 + sd / 2
        with Locations([(0, shelf_y, z + THICKNESS / 2)]):
            Box(WIDTH, sd, THICKNESS)

out_dir = Path(__file__).resolve().parent
part = shelf.part
export_step(part, out_dir / "wall_shelf_2plate.step")
export_stl(part, out_dir / "wall_shelf_2plate.stl")
print("Exported:", out_dir / "wall_shelf_2plate.step")
print("Exported:", out_dir / "wall_shelf_2plate.stl")
print("Bounding box:", part.bounding_box())
print("Shelf depths (mm):", [f"{d:.1f}" for d in shelf_depths])
