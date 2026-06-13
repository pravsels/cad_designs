# Arducam Top Mount

Adapts the SO-ARM100 overhead **webcam** camera-mount top to instead hold an
**Arducam** module. The original design ships as STL only, so we re-model it
parametrically (build123d -> STEP) using the reference meshes as dimensional
ground truth.

## Reference

Source: `SO-ARM100/Optional/Overhead_Cam_Mount_Webcam/` (STL only). Copies live
in `reference/`:

- `cam_mount_top.stl` — the part we are adapting (webcam clips into its round joint)
- `cam_mount_bottom.stl` — mates to the top via a linear joint; defines our interface
- `arm_base.stl` — clips into the bottom; context only
- `SOURCE_README.md` — upstream assembly guide

Re-measure any reference part with the shared tool:

```bash
.venv/bin/python ../tools/stl_inspect.py reference/cam_mount_top.stl
```

## Measured geometry (from `stl_inspect.py`)

Units mm. Meshes are axis-aligned. The mating interface we must preserve is the
`cam_mount_top` <-> `cam_mount_bottom` linear joint and screw pattern.

### `cam_mount_top` (the part to adapt)

- Overall: 25.4 (X) x 242.45 (Y) x 36.62 (Z); 6 through-holes (genus 6).
- Webcam round joint: two coaxial Z-bores, dia ~12.2, at (x, y) = (+/-4.58, 4.0).
  **This is the feature to replace with the Arducam interface.**
- Linear-joint screw holes (along X, through the 25.4 width): dia ~3.78 clearance
  with ~2.14 inner, at y=4.0, z=+/-14.15 (4 holes) — keep these to mate to the bottom.
- Far-end (y~230.65) small holes: 2.18 / 2.93 blind + 4.68 / 5.45 pockets.
- Body plates at x=+/-12.7 and z=+/-18.31.

- Recovery (`tools/recover_extrusion.py --axis x` -> STEP, scored with
  `tools/stl_compare.py --align none`): output in `reference/recovered/`.
  Median surface distance **0.59 mm**, **volume within 2.1%**, watertight. The
  body and the **X linear-joint screw holes** at `(y=4.0, z=+/-14.15)`, dia 3.78
  (the mate to `cam_mount_bottom`) reproduce faithfully. The only large localized
  error (Hausdorff 13.5 mm) is the **webcam round-joint Z-bores / end housing** —
  the feature we are replacing with the Arducam interface, so it is expected.

### `cam_mount_bottom` (bottom mate reference)

- Overall: 37.4 x 230.95 x 93.02; solid (genus 0).
- Big top bores along Z: dia 25.0 and 13.98 (cable/feature pass-through).
- Side M2 holes along X: dia ~1.49, at y=3.6/227 and z bands ~9-28.

### `arm_base` (context only)

- Overall: 89.6 x 7.2 x 159.04; thin clip plate (7.2 thick), 8 through-holes.
- Silhouette recovery (`tools/recover_extrusion.py` -> STEP -> STL) reproduces the
  outline and all holes to ~the sampling floor (median 0.355 mm). Its front-face
  step + groove pattern is decorative and **part-specific**; we intentionally do
  not model it. Recovery output in `reference/recovered/`.

## Plan

Geometry first, then holes, then the puzzle-fit (mating) interface — kept general,
not tuned to any one part's decorative detail.

### `cam_mount_top` parametric model (`cam_mount_top.py` -> `cam_mount_top.step`)

Hand-built in build123d from measured sections (replaces the silhouette recovery
for this part). Captures: filleted body bar (25.4 x 36.62, r1.5, L242.45);
joint-end H-channel (two side legs + central web, pockets cut from the +/-Z
faces); and the linear-joint **teardrop** M2 screw holes. The teardrops are
self-supporting print holes whose point aims **inward toward the centerline**
(z=0): neck circle ~dia 2.14 opening to ~3.78 at each face, apex ~1.4*r past the
circle.

The far end now replaces the webcam round joint with a centered Arducam B0283
end-face interface copied from `../cube_mount/cube_mount_arducam.py`: a
50 x 50 x 4 mm pad, 42.6 mm B0283 outline pocket, 2 mm pocket depth, and four
2.7 mm pegs at the measured B0283 mounting-hole positions.

### `cam_mount_bottom` reference

The bottom is not a generated deliverable for this design. Keep
`reference/cam_mount_bottom.stl` as the mating reference for the top mount's
linear joint and screw pattern.

Temporary Viewer comparison assets can be regenerated locally from the reference
and generated STLs; they are not part of the durable source for this pass.

## Status

- [x] Reference STLs imported and measured
- [x] `cam_mount_top` body + joint-end H-channel modeled parametrically
- [x] Linear-joint teardrop M2 screw holes (point inward toward center) modeled
- [x] Replace the 12.2 webcam round joint with the Arducam B0283 end-face interface
- [ ] Confirm the linear-joint screw pattern matches `cam_mount_bottom` (mating fit)
- [ ] Decide whether the original far-end tray holes remain useful with the Arducam pad

## Handoff (for the next agent)

Workflow that has been working well — measure from the STL, model parametrically,
score, then eyeball a render:

```bash
# build (writes STEP + STL, prints volume)
.venv/bin/python cam_mount_top.py --out cam_mount_top.step --stl-out /tmp/cmt.stl
# score against the reference (no alignment: both are in the same frame)
.venv/bin/python ../tools/stl_compare.py reference/cam_mount_top.stl /tmp/cmt.stl \
  --samples 60000 --align none
# shaded multi-angle render; --crop-axis y --crop-max 30 isolates the joint end
.venv/bin/python ../tools/render_mesh.py reference/cam_mount_top.stl /tmp/cmt.stl \
  --views 4 --elev 8 --out /tmp/cmp.png
```

build123d gotchas hit while writing `cam_mount_top.py`:
- Pass plain `float`s to `Polyline` — numpy scalars make `make_face` raise
  "No objects to create a hull".
- Don't wrap `BuildLine`/`make_face` in a helper function; the implicit builder
  context doesn't propagate, so keep those sketch blocks inline.

Open items, roughly in order: confirm the teardrop screw pattern actually mates
with `reference/cam_mount_bottom.stl`; refine the far-end pad if print/fit
testing needs a smaller support footprint; and model the original far-end tray
holes only if they remain useful with the Arducam interface.
