#!/usr/bin/env python3
"""Inspect STL meshes to recover the measurements needed to re-model them.

3D-print designs are frequently shared as STL only. STL is a triangle mesh with
no editable features, so the practical path is to measure the mesh and rebuild a
clean parametric model (build123d -> STEP). This tool extracts the facts that
make that re-modeling tractable:

- overall size, centroid, volume, surface area, watertightness, and genus
  (an estimate of the number of through-holes)
- axis-aligned planar-face inventory (reveals plate faces and their thicknesses)
- axis-aligned cylindrical-hole detection (center, diameter, depth)

It only assumes meshes are roughly axis-aligned, which holds for most printed
mounting parts. Run it on one or more STL files:

    python tools/stl_inspect.py reference/cam_mount_top.stl
    python tools/stl_inspect.py --format json reference/*.stl
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import trimesh
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix

AXES = ("x", "y", "z")


@dataclass
class PlanarFace:
    axis: str
    offset: float
    area: float
    face_count: int


@dataclass
class Hole:
    axis: str
    center: tuple[float, float]  # the two coords perpendicular to axis
    center_axes: tuple[str, str]
    diameter: float
    depth: float
    span: tuple[float, float]  # min/max along the hole axis
    kind: str = "hole"  # "hole" (void) or "boss" (protrusion), by wall normal direction
    through: bool = False  # span reaches both bounding faces along the axis


@dataclass
class Report:
    file: str
    triangles: int
    size_xyz: tuple[float, float, float]
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]
    centroid: tuple[float, float, float]
    volume: float
    area: float
    watertight: bool
    genus: int | None
    estimated_through_holes: int | None
    planar_faces: list[PlanarFace] = field(default_factory=list)
    holes: list[Hole] = field(default_factory=list)


def _r(value: float, places: int = 2) -> float:
    return round(float(value), places)


def _fit_circle(points: np.ndarray) -> tuple[float, float, float]:
    """Algebraic (Kasa) least-squares circle fit. Returns (cx, cy, radius)."""
    x = points[:, 0]
    y = points[:, 1]
    a = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x**2 + y**2
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy, c = sol
    radius = float(np.sqrt(c + cx**2 + cy**2))
    return float(cx), float(cy), radius


def _planar_faces(
    mesh: trimesh.Trimesh, *, normal_tol: float, min_area: float, limit: int
) -> list[PlanarFace]:
    """Group axis-aligned coplanar triangles into named planar faces."""
    normals = mesh.face_normals
    centers = mesh.triangles_center
    areas = mesh.area_faces
    results: list[PlanarFace] = []
    for axis_idx, axis in enumerate(AXES):
        aligned = np.abs(np.abs(normals[:, axis_idx]) - 1.0) < normal_tol
        if not aligned.any():
            continue
        offsets = np.round(centers[aligned, axis_idx], 2)
        face_areas = areas[aligned]
        for offset in np.unique(offsets):
            sel = offsets == offset
            total = float(face_areas[sel].sum())
            if total >= min_area:
                results.append(
                    PlanarFace(axis=axis, offset=_r(offset), area=_r(total), face_count=int(sel.sum()))
                )
    results.sort(key=lambda f: f.area, reverse=True)
    return results[:limit]


def _components(face_ids: np.ndarray, adjacency: np.ndarray) -> list[np.ndarray]:
    """Connected components among `face_ids` using shared-edge adjacency."""
    if len(face_ids) == 0:
        return []
    id_set = set(int(f) for f in face_ids)
    remap = {int(f): i for i, f in enumerate(face_ids)}
    edges = [
        (remap[int(a)], remap[int(b)])
        for a, b in adjacency
        if int(a) in id_set and int(b) in id_set
    ]
    n = len(face_ids)
    if edges:
        rows, cols = zip(*edges)
        data = np.ones(len(edges))
        graph = coo_matrix((data, (rows, cols)), shape=(n, n))
    else:
        graph = coo_matrix((n, n))
    count, labels = connected_components(graph, directed=False)
    return [face_ids[labels == k] for k in range(count)]


def _detect_holes(
    mesh: trimesh.Trimesh,
    *,
    wall_tol: float,
    radial_tol: float,
    circle_tol: float,
    min_faces: int,
    min_diameter: float,
    max_diameter: float,
) -> list[Hole]:
    """Detect axis-aligned cylindrical holes by fitting circles to wall faces."""
    normals = mesh.face_normals
    centers = mesh.triangles_center
    adjacency = mesh.face_adjacency
    bounds = mesh.bounds
    holes: list[Hole] = []
    for axis_idx, axis in enumerate(AXES):
        perp = [i for i in range(3) if i != axis_idx]
        # Wall faces of an axis-aligned cylinder have normals perpendicular to the axis.
        wall = np.abs(normals[:, axis_idx]) < wall_tol
        wall_ids = np.where(wall)[0]
        if len(wall_ids) < min_faces:
            continue
        for comp in _components(wall_ids, adjacency):
            if len(comp) < min_faces:
                continue
            pts = centers[comp][:, perp]
            cx, cy, radius = _fit_circle(pts)
            diameter = 2 * radius
            if not (min_diameter <= diameter <= max_diameter):
                continue
            # Confirm it is a real cylinder wall, not a prismatic wall (e.g. a box's
            # four sides) that happens to fit a circle: a true cylinder's wall
            # centroids sit on a tight circle (small residual), while a rectangle's
            # fitted circle has a large radial residual.
            dist = np.sqrt((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2)
            if np.std(dist) > circle_tol * max(radius, 1.0):
                continue
            radial = pts - np.array([cx, cy])
            radial /= np.linalg.norm(radial, axis=1, keepdims=True) + 1e-9
            face_perp_normals = normals[comp][:, perp]
            unit_n = face_perp_normals / (np.linalg.norm(face_perp_normals, axis=1, keepdims=True) + 1e-9)
            alignment = np.abs(np.sum(unit_n * radial, axis=1))
            if np.mean(alignment) < (1.0 - radial_tol):
                continue
            # Wall normals pointing toward the axis => void (hole); away => boss/peg.
            signed = float(np.mean(np.sum(unit_n * radial, axis=1)))
            kind = "boss" if signed > 0 else "hole"
            span = centers[comp][:, axis_idx]
            holes.append(
                Hole(
                    axis=axis,
                    center=(_r(cx), _r(cy)),
                    center_axes=(AXES[perp[0]], AXES[perp[1]]),
                    diameter=_r(diameter),
                    depth=_r(float(span.max() - span.min())),
                    span=(_r(float(span.min())), _r(float(span.max()))),
                    kind=kind,
                )
            )
    holes.sort(key=lambda h: (h.axis, h.center))
    return _merge_coaxial(holes, bounds=bounds)


def _merge_coaxial(
    holes: list[Hole], *, bounds: np.ndarray, center_tol: float = 0.6, dia_tol: float = 0.4
) -> list[Hole]:
    """Merge wall segments of one physical hole (same axis/center/diameter).

    A through-hole's wall is split into separate axial segments by the mesh
    (e.g. counterbore rings), so the raw detector reports each end separately.
    Group by axis + rounded center + rounded diameter and union their spans so a
    through-hole reads as a single feature with its full depth.
    """
    groups: dict[tuple, list[Hole]] = {}
    for h in holes:
        key = (
            h.axis,
            round(h.center[0] / center_tol),
            round(h.center[1] / center_tol),
            round(h.diameter / dia_tol),
        )
        groups.setdefault(key, []).append(h)
    merged: list[Hole] = []
    for members in groups.values():
        lo = min(m.span[0] for m in members)
        hi = max(m.span[1] for m in members)
        diameters = [m.diameter for m in members]
        ref = members[0]
        axis_idx = AXES.index(ref.axis)
        face_gap = 0.5
        through = bool(
            lo <= bounds[0][axis_idx] + face_gap and hi >= bounds[1][axis_idx] - face_gap
        )
        merged.append(
            Hole(
                axis=ref.axis,
                center=ref.center,
                center_axes=ref.center_axes,
                diameter=_r(float(np.median(diameters))),
                depth=_r(hi - lo),
                span=(lo, hi),
                kind=ref.kind,
                through=through,
            )
        )
    merged.sort(key=lambda h: (h.axis, h.center, h.diameter))
    return merged


def inspect(path: Path, args: argparse.Namespace) -> Report:
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{path} did not load as a single mesh")
    bounds = mesh.bounds
    size = bounds[1] - bounds[0]
    genus = None
    through = None
    if mesh.is_watertight:
        # Closed orientable surface: V - E + F = 2 - 2g; through-holes ~= genus.
        genus = int(round(1 - mesh.euler_number / 2))
        through = max(genus, 0)
    return Report(
        file=str(path),
        triangles=int(len(mesh.faces)),
        size_xyz=tuple(_r(v) for v in size),
        bbox_min=tuple(_r(v) for v in bounds[0]),
        bbox_max=tuple(_r(v) for v in bounds[1]),
        centroid=tuple(_r(v) for v in mesh.centroid),
        volume=_r(mesh.volume),
        area=_r(mesh.area),
        watertight=bool(mesh.is_watertight),
        genus=genus,
        estimated_through_holes=through,
        planar_faces=_planar_faces(
            mesh,
            normal_tol=args.normal_tol,
            min_area=args.min_face_area,
            limit=args.max_faces,
        ),
        holes=_detect_holes(
            mesh,
            wall_tol=args.wall_tol,
            radial_tol=args.radial_tol,
            circle_tol=args.circle_tol,
            min_faces=args.min_hole_faces,
            min_diameter=args.min_diameter,
            max_diameter=args.max_diameter,
        ),
    )


def _print_text(report: Report) -> None:
    print(f"== {report.file} ==")
    print(f"  triangles      : {report.triangles}")
    print(f"  size  (X,Y,Z)  : {report.size_xyz} mm")
    print(f"  bbox  min      : {report.bbox_min}")
    print(f"  bbox  max      : {report.bbox_max}")
    print(f"  centroid       : {report.centroid}")
    print(f"  volume         : {report.volume} mm^3")
    print(f"  surface area   : {report.area} mm^2")
    print(f"  watertight     : {report.watertight}")
    print(f"  genus          : {report.genus}")
    print(f"  ~through-holes : {report.estimated_through_holes}")
    print(f"  planar faces (axis @ offset mm, area mm^2):")
    for f in report.planar_faces:
        print(f"    {f.axis} @ {f.offset:>8}  area={f.area:>9}  ({f.face_count} tris)")
    print(f"  cylindrical features (kind, depth-type, axis, center, dia, depth):")
    if not report.holes:
        print("    (none detected with current thresholds)")
    for h in report.holes:
        ca = "".join(h.center_axes)
        depth_kind = "through" if h.through else "blind"
        print(
            f"    {h.kind:<4} [{depth_kind:>7}] axis {h.axis}  center({ca})={h.center}  "
            f"dia={h.diameter}  depth={h.depth}  span={h.span}"
        )
    print()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+", type=Path, help="STL file(s) to inspect")
    parser.add_argument("--format", choices=("text", "json"), default="text")
    parser.add_argument("--normal-tol", type=float, default=0.02, help="planar-face normal tolerance")
    parser.add_argument("--min-face-area", type=float, default=5.0, help="min planar-face area (mm^2) to report")
    parser.add_argument("--max-faces", type=int, default=20, help="max planar faces to report")
    parser.add_argument("--wall-tol", type=float, default=0.2, help="max |normal.axis| for a cylinder wall face")
    parser.add_argument("--radial-tol", type=float, default=0.15, help="radial-normal alignment tolerance")
    parser.add_argument("--circle-tol", type=float, default=0.08, help="max circle-fit residual / radius (rejects prismatic walls)")
    parser.add_argument("--min-hole-faces", type=int, default=6, help="min wall faces per detected hole")
    parser.add_argument("--min-diameter", type=float, default=1.0, help="min hole diameter (mm)")
    parser.add_argument("--max-diameter", type=float, default=60.0, help="max hole diameter (mm)")
    args = parser.parse_args(argv)

    reports = [inspect(p, args) for p in args.paths]
    if args.format == "json":
        print(json.dumps([asdict(r) for r in reports], indent=2))
    else:
        for r in reports:
            _print_text(r)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
