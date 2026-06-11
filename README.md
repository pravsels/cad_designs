# CAD Designs

Parametric CAD models built with [build123d](https://build123d.readthedocs.io/).

## Viewing

**1. Start the viewer once** (from the repo root; leave it running):

```bash
npm --prefix viewer run dev      # serves http://127.0.0.1:5173/
```

**2. Open a file** by URL. `dir` is the ABSOLUTE `models` path; `file` is relative
to it:

```
http://127.0.0.1:5173/?dir=<abs>/text-to-cad/models&file=cad_designs/<design>/<file>.stl
```

Example (run `echo $PWD/models` to get the absolute dir):

```
http://127.0.0.1:5173/?dir=/home/user/Desktop/code/text-to-cad/models&file=cad_designs/arducam_top_mount/reference/arm_base.stl
```

### Read this before debugging "it doesn't load"

- **`.stl` / `.glb` load instantly** with no extra step. For a quick look, open
  the `.stl` (every design exports one next to its `.step`).
- **`.step` renders from a hidden `.<name>.step.glb` sidecar.** The viewer builds
  it on first open using `.venv/bin/python` (build123d); this takes a few seconds
  and the canvas looks blank until it finishes. If a STEP stays blank, pre-build
  the sidecar (instant thereafter):

  ```bash
  .venv/bin/python models/cad_designs/tools/make_glb.py models/cad_designs/<design>/<file>.step
  ```

- **`dir` must be an absolute path**; `file` is relative to `dir`. A wrong/relative
  `dir` is the usual cause of an empty catalog.
- Sidecars (`.*.step.glb`) are git-ignored local caches; delete them to force a
  rebuild.

## Regenerating models

```bash
.venv/bin/python models/cad_designs/<design>/<script>.py
```

## Designs

| Directory | Description |
|-----------|-------------|
| `arducam_mount/` | Command-strip spacer slabs for Arducam B0283 pan-tilt kit |
| `arducam_top_mount/` | Overhead cam-mount top adapted from the SO-ARM100 webcam mount to hold an Arducam |
| `cube_mount/` | 63.5 mm cube mount block, with Arducam B0283 pocket variant |
| `leader_mount/` | Leader arm mount |
| `shelf/` | Shelf |

## Tools

`tools/stl_inspect.py` measures STL-only designs (common for shared 3D prints) so
they can be re-modeled parametrically. It reports size, volume, watertightness,
genus (through-hole count), an axis-aligned planar-face inventory (plate
thicknesses and feature floor levels), and axis-aligned cylindrical-feature
detection labelled `hole` vs `boss` and `through` vs `blind`:

```bash
.venv/bin/python models/cad_designs/tools/stl_inspect.py path/to/part.stl
.venv/bin/python models/cad_designs/tools/stl_inspect.py --format json path/to/*.stl
```

Validated against `arducam_mount` parts (STL vs known build123d source): bounding
box, feature centers, diameters, hole/boss classification, and genus came back
exact. Known limitations:

- **Blind-hole depth is a lower bound** (coarse tessellation truncates the wall).
  Read exact blind depths from the planar-face floor offset (e.g. a socket floor
  shows up as a small planar face at its z level).
- **through/blind is heuristic**; trust `genus` for the true through-hole count.
- Detection assumes **axis-aligned** features; angled holes are not fit.

`tools/stl_compare.py` scores how well one mesh reconstructs another via symmetric
KD-tree Chamfer surface distance (mean/RMS/percentile/Hausdorff, plus volume
delta). Intended for the round trip: original STL -> hand-built STEP -> derived
STL.

```bash
.venv/bin/python models/cad_designs/tools/stl_compare.py original.stl derived.stl
```

Caveats:

- **Coordinate frame matters.** Distance is only a reconstruction score if both
  meshes share a frame. Re-modeling in the reference's coordinates keeps them
  aligned (`--align none`, default). For arbitrary external meshes, register
  first with `--align centroid|icp`; a bad ICP minimum inflates the score, so
  check the reported cost.
- **Sampling floor.** Even a mesh vs itself reports a small non-zero mean; run
  reference-vs-itself first to calibrate, and treat results near that floor as
  identical. Raise `--samples` to lower the floor.
- **Human fallback when alignment is uncertain.** If ICP cost is high/ambiguous
  or the part is symmetric or featureless, do not trust the raw score. Ask the
  user to qualitatively confirm the meshes are aligned (e.g. overlay both in the
  CAD Viewer) before reading the distance as a reconstruction quality measure.

`tools/recover_extrusion.py` recovers an editable extruded STEP from a plate-like
STL by tracing the flat-face boundary loops (outer silhouette + hole rims) and
re-extruding in the STL's own frame. Pairs with `stl_compare.py --align none`:

```bash
.venv/bin/python models/cad_designs/tools/recover_extrusion.py part.stl \
  --out recovered.step --stl-out /tmp/derived.stl
```

Default `--mode silhouette` recovers a single constant-thickness extrusion with
through-holes (the general geometry + holes pass). For parts whose mate is a
genuine depth step, `--mode multilevel` reconstructs discrete top/bottom depth
levels (significant levels only; `--min-level-frac` filters slivers). Neither
mode recovers chamfers, draft, or freeform surfaces; those surface as a volume
delta and localized Hausdorff error in `stl_compare` — finish such parts by hand.
