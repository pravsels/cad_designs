#!/usr/bin/env python3
"""Pre-generate the CAD Viewer GLB sidecar for STEP file(s).

The viewer renders a `.step` from a hidden `.<name>.step.glb` sidecar next to it.
It builds that sidecar on first open (via .venv build123d), which can take a few
seconds and look blank meanwhile. Run this first to make STEP files render
instantly:

    .venv/bin/python models/cad_designs/tools/make_glb.py path/to/model.step [more.step ...]

The sidecar is git-ignored (.gitignore: .*.step.glb), so it is a local cache.
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import export_gltf, import_step


def make(step: Path) -> Path:
    sidecar = step.parent / f".{step.name}.glb"
    export_gltf(import_step(str(step)), str(sidecar), binary=True)
    return sidecar


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    for arg in argv:
        step = Path(arg)
        if step.suffix.lower() not in (".step", ".stp"):
            print(f"skip (not a STEP): {step}")
            continue
        print("wrote", make(step))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
