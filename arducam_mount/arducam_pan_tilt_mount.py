"""
Command-strip spacer slabs for Arducam B0283 Pan-Tilt Kit.

Base slab: flat bottom for the table strip, top sockets for stacking.
Stack slab: bottom pegs plug into sockets below, top sockets accept another slab.
Slab body height = 10mm; bottom pegs sink into the sockets below when stacked.
"""

from pathlib import Path

from build123d import *

PLATE_W, PLATE_D, PLATE_T = 30.5, 42.5, 10

PEG_X, PEG_Y = 10, 16
PEG_D = 2.4
PEG_H = 4.5
SOCKET_D = 2.8
SOCKET_DEPTH = 5.0


def add_top_sockets() -> None:
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            with Locations((sx * PEG_X, sy * PEG_Y, PLATE_T - SOCKET_DEPTH)):
                Cylinder(
                    SOCKET_D / 2,
                    SOCKET_DEPTH,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT,
                )


def build_base_slab():
    with BuildPart() as slab:
        Box(PLATE_W, PLATE_D, PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        add_top_sockets()
    return slab.part


def build_stack_slab():
    with BuildPart() as slab:
        Box(PLATE_W, PLATE_D, PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        add_top_sockets()

        for sx in [-1, 1]:
            for sy in [-1, 1]:
                with Locations((sx * PEG_X, sy * PEG_Y, -PEG_H)):
                    Cylinder(
                        PEG_D / 2,
                        PEG_H,
                        align=(Align.CENTER, Align.CENTER, Align.MIN),
                    )
    return slab.part


out_dir = Path(__file__).resolve().parent
exports = {
    "arducam_pan_tilt_mount_base": build_base_slab(),
    "arducam_pan_tilt_mount_stack": build_stack_slab(),
}

for name, part in exports.items():
    step_path = out_dir / f"{name}.step"
    stl_path = out_dir / f"{name}.stl"
    export_step(part, step_path)
    export_stl(part, stl_path)
    print("Exported:", step_path)
    print("Exported:", stl_path)
    print("Bounding box:", part.bounding_box())
