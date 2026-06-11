#!/usr/bin/env python3
"""Render clean multi-angle pictures of mesh(es) headlessly (pyrender + EGL).

Use this to actually *see* a part from several angles instead of guessing from
cross-sections. Pass one or more STLs; each is rendered from the same set of
azimuths and stacked as a row, so original-vs-reconstruction comparisons line up.

    PYOPENGL_PLATFORM=egl .venv/bin/python tools/render_mesh.py a.stl b.stl \
        --out /tmp/cmp.png --views 4 --elev 20 --crop-axis y --crop-max 16
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import numpy as np
import trimesh
import pyrender
from PIL import Image

AXES = {"x": 0, "y": 1, "z": 2}


def _look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    pose = np.eye(4)
    pose[:3, 0] = s
    pose[:3, 1] = u
    pose[:3, 2] = -f
    pose[:3, 3] = eye
    return pose


def render_one(mesh: trimesh.Trimesh, az_deg: float, elev_deg: float, size: int) -> Image.Image:
    scene = pyrender.Scene(bg_color=[245, 245, 248, 255], ambient_light=[0.35, 0.35, 0.35])
    scene.add(pyrender.Mesh.from_trimesh(mesh, smooth=False))
    center = mesh.bounds.mean(axis=0)
    radius = float(np.linalg.norm(mesh.extents) / 2)
    yfov = np.radians(45)
    dist = radius / np.sin(yfov / 2) * 1.25
    az, el = np.radians(az_deg), np.radians(elev_deg)
    direction = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    eye = center + direction * dist
    pose = _look_at(eye, center)
    scene.add(pyrender.PerspectiveCamera(yfov=yfov), pose=pose)
    scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=3.0), pose=pose)
    r = pyrender.OffscreenRenderer(size, size)
    color, _ = r.render(scene)
    r.delete()
    return Image.fromarray(color)


def render_grid(meshes, labels, outfile, views, elev, size):
    azimuths = list(np.linspace(-60, 240, views, endpoint=True)) if views > 1 else [-60]
    rows = []
    for mesh, label in zip(meshes, labels):
        imgs = [render_one(mesh, az, elev, size) for az in azimuths]
        row = Image.new("RGB", (size * len(imgs), size + 22), "white")
        for i, im in enumerate(imgs):
            row.paste(im, (i * size, 22))
        from PIL import ImageDraw
        ImageDraw.Draw(row).text((4, 4), label, fill="black")
        rows.append(row)
    grid = Image.new("RGB", (rows[0].width, sum(r.height for r in rows)), "white")
    y = 0
    for r in rows:
        grid.paste(r, (0, y))
        y += r.height
    Path(outfile).parent.mkdir(parents=True, exist_ok=True)
    grid.save(outfile)
    print(f"saved {outfile}  ({len(meshes)} mesh(es) x {len(azimuths)} views)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stls", type=Path, nargs="+", help="one or more STL files (rows)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--views", type=int, default=4, help="azimuths per row")
    ap.add_argument("--elev", type=float, default=20.0)
    ap.add_argument("--size", type=int, default=480)
    ap.add_argument("--crop-axis", choices=list(AXES), default=None, help="keep only faces below crop-max on this axis")
    ap.add_argument("--crop-max", type=float, default=None)
    args = ap.parse_args(argv)

    meshes, labels = [], []
    for p in args.stls:
        m = trimesh.load(p, force="mesh")
        if args.crop_axis is not None and args.crop_max is not None:
            ax = AXES[args.crop_axis]
            fmask = (m.vertices[m.faces][:, :, ax] < args.crop_max).all(axis=1)
            m = m.submesh([np.where(fmask)[0]], append=True)
        meshes.append(m)
        labels.append(p.name)
    render_grid(meshes, labels, args.out, args.views, args.elev, args.size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
