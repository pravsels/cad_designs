# Arducam Pan-Tilt Mount

Stackable command-strip spacer slabs for the Arducam B0283 Pan-Tilt Kit.

## Files

- `arducam_pan_tilt_mount.py` — build123d source
- `arducam_pan_tilt_mount_base.step` / `.stl` — flat-bottom base slab
- `arducam_pan_tilt_mount_stack.step` / `.stl` — stack slab with bottom pegs

Regenerate: `.venv/bin/python models/arducam-mount/arducam_pan_tilt_mount.py`

## Design

30.5 × 42.5 × 10 mm slab matching the measured camera-base footprint.
Print one base slab, then print as many stack slabs as needed.

- Base slab: flat bottom for the table command strip, top sockets for stacking.
- Stack slab: bottom pegs plug into the slab below, top sockets accept another slab.
- Keep the center faces clear for command strips.

## Decisions (don't undo without reason)

- **Command strips, not screws** — one strip attaches the base slab to
  the table, another attaches the camera base to the top slab.
- **Two variants** — the bottom-most slab needs a smooth bottom; stack
  slabs need bottom pegs.
- **Round corner pegs/sockets** — four pegs resist rotation when stacked
  while keeping the command-strip area mostly clear.

## Unknowns

Peg/socket clearance is an initial print-fit estimate. Test one base
and one stack slab before printing a full stack.
