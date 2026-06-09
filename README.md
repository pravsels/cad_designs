# CAD Designs

Parametric CAD models built with [build123d](https://build123d.readthedocs.io/).

## Viewing

Start the Viewer from the repo root (Vite dev mode):

```bash
npm --prefix viewer run dev
```

Then open a model at `http://127.0.0.1:5173/` with query params:

```
http://127.0.0.1:5173/?dir=/absolute/path/to/text-to-cad/models&file=cad_designs/<design>/<file>.step
```

Example:

```
http://127.0.0.1:5173/?dir=$PWD/models&file=cad_designs/cube_mount/cube_mount_arducam.step
```

## Regenerating models

```bash
.venv/bin/python models/cad_designs/<design>/<script>.py
```

## Designs

| Directory | Description |
|-----------|-------------|
| `arducam_mount/` | Command-strip spacer slabs for Arducam B0283 pan-tilt kit |
| `cube_mount/` | 63.5 mm cube mount block, with Arducam B0283 pocket variant |
| `leader_mount/` | Leader arm mount |
| `shelf/` | Shelf |
