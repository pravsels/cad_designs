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
circle. Scored vs `reference/cam_mount_top.stl` (`stl_compare --align none`):
median **0.37 mm**, volume delta **8%** — concentrated in the still-unmodeled
webcam round joint (to be replaced) and the far-end tray.

### `cam_mount_bottom` parametric model (`cam_mount_bottom.py` -> `cam_mount_bottom.step`)

Interface-first reconstruction from measured sections. Captures: overall bbox
(37.4 x 230.95 x 93.02); chamfered long body planes; joint-end envelope; far-end
slotted frame; and measured side M2 blind-hole positions. The clipped round
pass-through wall near y=3.6 is still approximate because the STL's 25.0 /
13.98 mm cylindrical detections are not a simple open cylinder in the end
envelope. Scored vs `reference/cam_mount_bottom.stl`
(`stl_compare --align none --samples 60000`): median **0.388 mm**, mean
**0.543 mm**, 95th **0.953 mm**, volume delta **0.24%**. A measured waisted
joint-side profile was tested and rejected because it removed too much material;
the remaining extra joint-end plate is the next detail to solve with a better
feature model.

Temporary Viewer comparison assets can be regenerated locally from the reference
and generated STLs; they are not part of the durable source for this pass.

## Status

- [x] Reference STLs imported and measured
- [x] `cam_mount_top` body + joint-end H-channel modeled parametrically
- [x] Linear-joint teardrop M2 screw holes (point inward toward center) modeled
- [x] `cam_mount_bottom` interface-first body + joint envelope modeled parametrically
- [x] Original/generated bottom and four-part top/bottom Viewer comparisons generated locally
- [ ] Measure + model the far-end (y~230) tray + holes (2.18/2.93/4.68/5.45)
- [ ] Confirm the linear-joint screw pattern matches `cam_mount_bottom` (mating fit)
- [ ] Replace the 12.2 webcam round joint with the Arducam module interface
- [x] Export bottom STEP + STL, score against `reference/cam_mount_bottom.stl`

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
# regenerate the viewer GLB sidecar after editing the STEP (then hard-refresh viewer)
.venv/bin/python ../tools/make_glb.py cam_mount_top.step
```

Bottom build + review:

```bash
.venv/bin/python cam_mount_bottom.py --out cam_mount_bottom.step --stl-out cam_mount_bottom.stl
.venv/bin/python ../tools/stl_compare.py reference/cam_mount_bottom.stl cam_mount_bottom.stl \
  --samples 60000 --align none
```

build123d gotchas hit while writing `cam_mount_top.py`:
- Pass plain `float`s to `Polyline` — numpy scalars make `make_face` raise
  "No objects to create a hull".
- Don't wrap `BuildLine`/`make_face` in a helper function; the implicit builder
  context doesn't propagate, so keep those sketch blocks inline.

Open items, roughly in order: inspect regenerated original/generated bottom and
four-part top/bottom comparisons; refine the bottom far-end frame detail called
out during review; confirm the teardrop screw pattern actually mates with
`cam_mount_bottom`; far-end (y~230) tray + its 4 holes on the top; then replace
the webcam round Z-joint with the Arducam module interface (that's the whole
point of this part). The top's 8% volume delta is almost entirely that unmodeled
joint + tray.
