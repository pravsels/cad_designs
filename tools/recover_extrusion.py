#!/usr/bin/env python3
"""Recover an editable extruded B-rep (STEP) from a plate-like (2.5D) STL.

Many shared 3D-print parts are 2.5D: a flat-ish plate along one thin axis whose
top and bottom surfaces sit at a few discrete depth levels (steps, recesses,
grooves, bosses, pegs), pierced by holes. For those we can recover a *clean,
editable* STEP straight from the mesh (not a triangle wrapper), built in the
STL's own coordinate frame so it scores directly with `stl_compare.py --align none`.

Approach (general, no per-part feature code):
- Pick the thin (extrusion) axis. Let `gmin` be the global min along it.
- Every face whose normal points along +axis is a piece of the TOP surface; group
  these into levels by height. Every -axis face is part of the BOTTOM surface;
  group into levels by depth.
- TopSolid = union over top levels h of (top footprint at h) extruded gmin..h.
- Carve = union over bottom levels d of (bottom footprint at d) extruded gmin..d.
- solid = TopSolid - Carve. This reproduces each column's [bottom, top] interval,
  so steps/recesses/bosses/pegs/holes all fall out of the same logic.

Modes:
- `multilevel` (default): the full 2.5D reconstruction above.
- `silhouette`: legacy single-level (outer profile + through-holes) for a fast
  general-shape pass or when multilevel booleans struggle.

Limitations (verify with stl_compare): assumes one solid interval per column
(true 2.5D); does not recover chamfers, draft, or freeform surfaces. Profile
fidelity is the mesh tessellation (curves become polylines).

    python tools/recover_extrusion.py part.stl --out recovered.step --stl-out out.stl
    python tools/recover_extrusion.py part.stl --mode silhouette --out recovered.step
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh
from matplotlib.path import Path as MplPath
from build123d import (
    BuildLine,
    BuildPart,
    BuildSketch,
    Mode,
    Plane,
    Polyline,
    export_step,
    export_stl,
    extrude,
    make_face,
)

AXES = ("x", "y", "z")
SKETCH_PLANE = {"x": Plane.YZ, "y": Plane.XZ, "z": Plane.XY}
# The two in-plane coordinate axes for each extrusion axis (match the sketch plane).
PLANE_AXES = {"x": (1, 2), "y": (0, 2), "z": (0, 1)}


def _trace_loops(faces: np.ndarray) -> list[list[int]]:
    """Order the boundary edges of a face set into closed vertex loops."""
    count: dict[tuple[int, int], int] = defaultdict(int)
    for f in faces:
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            count[tuple(sorted((int(a), int(b))))] += 1
    boundary = [edge for edge, n in count.items() if n == 1]
    adjacency: dict[int, list[int]] = defaultdict(list)
    for a, b in boundary:
        adjacency[a].append(b)
        adjacency[b].append(a)
    unused = set(boundary)
    loops: list[list[int]] = []
    while unused:
        a, b = next(iter(unused))
        unused.discard((a, b))
        loop = [a, b]
        while True:
            nxt = [n for n in adjacency[loop[-1]] if tuple(sorted((loop[-1], n))) in unused]
            if not nxt:
                break
            n = nxt[0]
            unused.discard(tuple(sorted((loop[-1], n))))
            loop.append(n)
            if n == loop[0]:
                break
        loops.append(loop)
    loops.sort(key=len, reverse=True)
    return loops


def _loop_to_2d(loop: list[int], verts: np.ndarray, ij: tuple[int, int]) -> list[tuple[float, float]]:
    i, j = ij
    pts = [(float(verts[v, i]), float(verts[v, j])) for v in loop]
    if pts and pts[-1] == pts[0]:
        pts = pts[:-1]
    out = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - out[-1][0]) > 1e-6 or abs(p[1] - out[-1][1]) > 1e-6:
            out.append(p)
    return out


def _poly_area(poly: list[tuple[float, float]]) -> float:
    arr = np.asarray(poly)
    x, y = arr[:, 0], arr[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def _classify_nesting(polys: list[list[tuple[float, float]]]) -> list[tuple[list, bool]]:
    """Tag each loop as solid (even nesting depth) or hole (odd) via containment."""
    paths = [MplPath(np.asarray(p)) for p in polys]
    reps = [np.asarray(p).mean(axis=0) for p in polys]
    tagged = []
    for i, poly in enumerate(polys):
        depth = sum(1 for j, path in enumerate(paths) if j != i and path.contains_point(reps[i]))
        tagged.append((poly, depth % 2 == 1))
    return tagged


def _cluster_levels(offsets: np.ndarray, tol: float) -> list[float]:
    """Cluster face offsets along the axis into discrete level heights."""
    if len(offsets) == 0:
        return []
    s = np.sort(offsets)
    levels = [[s[0]]]
    for v in s[1:]:
        if v - levels[-1][-1] <= tol:
            levels[-1].append(v)
        else:
            levels.append([v])
    return [float(np.median(group)) for group in levels]


def _slab(axis: str, polys_tagged: list[tuple[list, bool]], amount: float):
    """Build a solid of the given (holed) footprint spanning [0, amount] on axis."""
    if amount <= 1e-6:
        return None
    plane = SKETCH_PLANE[axis]
    axis_idx = AXES.index(axis)
    with BuildPart() as bp:
        with BuildSketch(plane):
            for poly, is_hole in sorted(polys_tagged, key=lambda t: _poly_area(t[0]), reverse=True):
                with BuildLine():
                    Polyline(*poly, close=True)
                make_face(mode=Mode.SUBTRACT if is_hole else Mode.ADD)
        extrude(amount=amount)
    part = bp.part
    # Normalize so the slab spans [0, amount] regardless of the plane's extrude sign.
    lo_axis = getattr(part.bounding_box().min, axis.upper())
    if abs(lo_axis) > 1e-9:
        delta = [0.0, 0.0, 0.0]
        delta[axis_idx] = -lo_axis
        part = part.translate(tuple(delta))
    return part


def _side_level_areas(faces, mask, centers, areas, axis_idx, plane_tol):
    """Return [(height, total_area, near_mask)] for clustered levels on one side."""
    out = []
    for h in _cluster_levels(centers[mask, axis_idx], plane_tol):
        near = mask & (np.abs(centers[:, axis_idx] - h) <= plane_tol)
        out.append((h, float(areas[near].sum()), near))
    return out


def _footprints(faces, verts, near, ij, *, min_area):
    polys = [_loop_to_2d(lp, verts, ij) for lp in _trace_loops(faces[near])]
    polys = [p for p in polys if len(p) >= 3 and _poly_area(p) >= min_area]
    return _classify_nesting(polys) if polys else None


def recover(
    stl_path: Path,
    out_step: Path,
    *,
    axis: str | None,
    out_stl: Path | None,
    mode: str = "silhouette",
    plane_tol: float = 0.3,
    min_area: float = 1.0,
    min_level_frac: float = 0.05,
) -> dict:
    mesh = trimesh.load(stl_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{stl_path} did not load as a single mesh")
    verts, faces, normals = mesh.vertices, mesh.faces, mesh.face_normals
    centers = mesh.triangles_center
    face_areas = mesh.area_faces
    if axis is None:
        axis = AXES[int(np.argmin(mesh.extents))]
    axis_idx = AXES.index(axis)
    ij = PLANE_AXES[axis]
    gmin = float(mesh.bounds[0][axis_idx])
    gmax = float(mesh.bounds[1][axis_idx])

    if mode == "silhouette":
        thickness = gmax - gmin
        on_face = (normals[:, axis_idx] < -0.9) & (centers[:, axis_idx] < gmin + 0.5 * thickness)
        loops = [_loop_to_2d(lp, verts, ij) for lp in _trace_loops(faces[on_face])]
        loops = [p for p in loops if len(p) >= 3]
        tagged = _classify_nesting(loops)
        solid = _slab(axis, tagged, thickness)
        levels_info = {"top": 1, "bottom": 0}
    else:
        up = normals[:, axis_idx] > 0.9
        dn = normals[:, axis_idx] < -0.9
        top_raw = _side_level_areas(faces, up, centers, face_areas, axis_idx, plane_tol)
        bot_raw = _side_level_areas(faces, dn, centers, face_areas, axis_idx, plane_tol)
        # Keep only significant depth levels; tiny slivers (hole seats, fine
        # grooves) are deferred to later phases and would also wreck the booleans.
        max_area = max((a for _, a, _ in top_raw + bot_raw), default=0.0)
        thresh = min_level_frac * max_area
        top_levels = [(h, m) for h, a, m in top_raw if a >= thresh]
        bot_levels = [(h, m) for h, a, m in bot_raw if a >= thresh]
        if not top_levels:
            raise ValueError("no significant top (+axis) levels; try --axis or --mode silhouette")
        top_solid = None
        for h, near in top_levels:
            tagged = _footprints(faces, verts, near, ij, min_area=min_area)
            slab = _slab(axis, tagged, h - gmin) if tagged else None
            if slab is not None:
                top_solid = slab if top_solid is None else top_solid + slab
        carve = None
        for d, near in bot_levels:
            tagged = _footprints(faces, verts, near, ij, min_area=min_area)
            slab = _slab(axis, tagged, d - gmin) if tagged else None
            if slab is not None:
                carve = slab if carve is None else carve + slab
        solid = top_solid if carve is None else top_solid - carve
        levels_info = {"top": len(top_levels), "bottom": len(bot_levels)}

    # Reposition to the original frame along the axis.
    bb = solid.bounding_box()
    delta = [0.0, 0.0, 0.0]
    delta[axis_idx] = gmin - getattr(bb.min, axis.upper())
    solid = solid.translate(tuple(delta))

    out_step.parent.mkdir(parents=True, exist_ok=True)
    export_step(solid, str(out_step))
    if out_stl is not None:
        export_stl(solid, str(out_stl))
    return {
        "axis": axis,
        "mode": mode,
        "thickness": round(gmax - gmin, 3),
        "levels": levels_info,
        "volume": round(float(solid.volume), 2),
        "step": str(out_step),
        "stl": None if out_stl is None else str(out_stl),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stl", type=Path, help="plate-like STL to recover")
    parser.add_argument("--out", type=Path, required=True, help="output STEP path")
    parser.add_argument("--stl-out", type=Path, default=None, help="also write a derived STL (for scoring)")
    parser.add_argument("--axis", choices=AXES, default=None, help="extrusion axis (default: thinnest)")
    parser.add_argument("--mode", choices=("silhouette", "multilevel"), default="silhouette",
                        help="silhouette = outer profile + through-holes (default); "
                             "multilevel = full 2.5D depth levels (for genuine mating steps)")
    parser.add_argument("--plane-tol", type=float, default=0.3, help="level clustering tolerance (mm)")
    parser.add_argument("--min-area", type=float, default=1.0, help="ignore footprint regions below this area (mm^2)")
    parser.add_argument("--min-level-frac", type=float, default=0.05,
                        help="ignore depth levels whose total area is below this fraction of the largest level")
    args = parser.parse_args(argv)

    info = recover(
        args.stl, args.out, axis=args.axis, out_stl=args.stl_out,
        mode=args.mode, plane_tol=args.plane_tol, min_area=args.min_area,
        min_level_frac=args.min_level_frac,
    )
    print(f"recovered: axis={info['axis']} mode={info['mode']} thickness={info['thickness']} mm")
    print(f"  levels: top={info['levels']['top']} bottom={info['levels']['bottom']}")
    print(f"  volume: {info['volume']} mm^3")
    print(f"  wrote: {info['step']}" + (f" , {info['stl']}" if info["stl"] else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
