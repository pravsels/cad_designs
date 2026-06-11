#!/usr/bin/env python3
"""Score how well one mesh reconstructs another (symmetric surface distance).

Intended for the re-modeling round trip: original STL -> hand-built STEP ->
derived STL. Comparing the original and derived meshes gives an objective
reconstruction score instead of eyeballing features.

IMPORTANT - coordinate frame: surface distance is only a reconstruction score if
both meshes share a frame. A perfect duplicate that is translated/rotated/scaled
differently scores badly. Two regimes:

- `--align none` (default): assume a shared frame. Use this for the re-modeling
  round trip, where the STEP is modeled in the reference's coordinates, so a high
  score genuinely means geometric mismatch.
- `--align centroid|icp`: register first for arbitrary external meshes. `icp`
  does coarse + iterative-closest-point fitting. Caveat: a bad ICP minimum
  (common with symmetry) inflates the score, conflating misalignment with a poor
  reconstruction; check the reported alignment cost.

When automated alignment is unreliable (high/ambiguous ICP cost, symmetric or
featureless parts), do NOT trust the raw number. Fall back to a human check:
ask the user to confirm the two meshes are aligned/overlaid (e.g. load both in
the CAD Viewer), and only then read the distance as a reconstruction score.
Future agents: prefer asking for a quick qualitative confirmation over reporting
a confident-but-wrong score.

Metric: sample points on each surface, measure the nearest-surface distance to
the other mesh in both directions, and combine them (symmetric). Reports:

- mean / RMS / median symmetric surface distance (mm)
- 95th percentile and Hausdorff (max) distance (mm)
- the same normalized by the reference bounding-box diagonal (%)
- volume delta for watertight meshes

Sampling floor: point-to-point Chamfer has a noise floor set by inter-sample
spacing, so even a mesh compared with itself reports a small non-zero mean.
Calibrate by running the tool on the reference vs itself first; a reconstruction
whose mean approaches that floor is effectively identical. Raise --samples to
lower the floor. The percentile/Hausdorff numbers localize the worst mismatch
(often a single chamfer or fillet).

    python tools/stl_compare.py original.stl derived.stl
    python tools/stl_compare.py --samples 200000 --format json a.stl b.stl
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree


@dataclass
class CompareResult:
    reference: str
    other: str
    align: str
    align_cost: float | None
    samples: int
    mean: float
    rms: float
    median: float
    p95: float
    hausdorff: float
    bbox_diagonal: float
    mean_pct: float
    hausdorff_pct: float
    reference_volume: float | None
    other_volume: float | None
    volume_delta_pct: float | None


def _load(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{path} did not load as a single mesh")
    return mesh


def _sample(mesh: trimesh.Trimesh, n: int, rng: np.random.Generator) -> np.ndarray:
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=int(rng.integers(0, 2**31 - 1)))
    return np.asarray(pts)


def _chamfer(a_pts: np.ndarray, b_pts: np.ndarray) -> np.ndarray:
    """Symmetric nearest-point distances between two surface point clouds.

    Point-to-point Chamfer (KD-tree) approximates surface distance and needs only
    scipy; dense sampling keeps the small point-to-surface bias negligible.
    """
    d_ab, _ = cKDTree(b_pts).query(a_pts)
    d_ba, _ = cKDTree(a_pts).query(b_pts)
    return np.concatenate([d_ab, d_ba])


def _kabsch(p: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid transform (R, t) minimizing ||R*p + t - q|| for paired points."""
    cp, cq = p.mean(0), q.mean(0)
    h = (p - cp).T @ (q - cq)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return r, cq - r @ cp


def _icp(src: np.ndarray, dst: np.ndarray, *, iters: int = 40) -> tuple[np.ndarray, float]:
    """Point-to-point ICP. Returns (4x4 transform applied to src, mean residual)."""
    tree = cKDTree(dst)
    transform = np.eye(4)
    cur = src.copy()
    last = np.inf
    for _ in range(iters):
        dist, idx = tree.query(cur)
        r, t = _kabsch(cur, dst[idx])
        cur = (r @ cur.T).T + t
        step = np.eye(4)
        step[:3, :3], step[:3, 3] = r, t
        transform = step @ transform
        mean_d = float(dist.mean())
        if abs(last - mean_d) < 1e-6:
            break
        last = mean_d
    dist, _ = tree.query(cur)
    return transform, float(dist.mean())


def _register(
    ref: trimesh.Trimesh, oth: trimesh.Trimesh, mode: str, rng: np.random.Generator
) -> tuple[trimesh.Trimesh, float | None]:
    """Move `oth` onto `ref` for pose-invariant comparison. Returns (moved, cost)."""
    if mode == "none":
        return oth, None
    moved = oth.copy()
    if mode == "centroid":
        moved.apply_translation(ref.centroid - moved.centroid)
        return moved, None
    if mode == "icp":
        # Seed ICP from several principal-axis orientations (PCA sign ambiguity)
        # so symmetry is less likely to trap it in a bad minimum; keep the best.
        # NOTE: ICP can still settle on a wrong pose. If the returned cost is high
        # or ambiguous, treat the distance score as unreliable and fall back to a
        # human visual alignment check (see module docstring) rather than trusting
        # the number.
        ref_pts = _sample(ref, 3000, rng)
        oth_pts = _sample(moved, 3000, rng)
        rc, oc = ref_pts.mean(0), oth_pts.mean(0)
        ref_axes = np.linalg.svd(ref_pts - rc)[2]
        oth_axes = np.linalg.svd(oth_pts - oc)[2]
        best_t, best_cost = np.eye(4), np.inf
        seeds = [np.eye(4)]
        for sx in (1, -1):
            for sy in (1, -1):
                sz = sx * sy  # keep a proper rotation (det = +1)
                rot = ref_axes.T @ np.diag([sx, sy, sz]) @ oth_axes
                if np.linalg.det(rot) < 0:
                    continue
                seed = np.eye(4)
                seed[:3, :3] = rot
                seed[:3, 3] = rc - rot @ oc
                seeds.append(seed)
        for seed in seeds:
            seeded = (seed[:3, :3] @ oth_pts.T).T + seed[:3, 3]
            refine, cost = _icp(seeded, ref_pts)
            if cost < best_cost:
                best_cost, best_t = cost, refine @ seed
        moved.apply_transform(best_t)
        return moved, float(best_cost)
    raise ValueError(f"unknown align mode: {mode}")


def compare(reference: Path, other: Path, *, samples: int, align: str = "none", seed: int = 0) -> CompareResult:
    ref = _load(reference)
    oth = _load(other)
    rng = np.random.default_rng(seed)
    oth, align_cost = _register(ref, oth, align, rng)

    both = _chamfer(_sample(ref, samples, rng), _sample(oth, samples, rng))

    diag = float(np.linalg.norm(ref.extents))
    mean = float(both.mean())
    hausdorff = float(both.max())

    ref_vol = float(ref.volume) if ref.is_watertight else None
    oth_vol = float(oth.volume) if oth.is_watertight else None
    vol_delta = (
        abs(ref_vol - oth_vol) / ref_vol * 100.0
        if ref_vol not in (None, 0.0) and oth_vol is not None
        else None
    )

    def _r(value: float | None, places: int = 4) -> float | None:
        return None if value is None else round(value, places)

    return CompareResult(
        reference=str(reference),
        other=str(other),
        align=align,
        align_cost=_r(align_cost),
        samples=samples,
        mean=_r(mean),
        rms=_r(float(np.sqrt((both**2).mean()))),
        median=_r(float(np.median(both))),
        p95=_r(float(np.percentile(both, 95))),
        hausdorff=_r(hausdorff),
        bbox_diagonal=_r(diag),
        mean_pct=_r(mean / diag * 100.0 if diag else None),
        hausdorff_pct=_r(hausdorff / diag * 100.0 if diag else None),
        reference_volume=_r(ref_vol, 2),
        other_volume=_r(oth_vol, 2),
        volume_delta_pct=_r(vol_delta, 3),
    )


def _print_text(r: CompareResult) -> None:
    print(f"reference : {r.reference}")
    print(f"other     : {r.other}")
    print(f"align     : {r.align}" + (f"  (icp cost {r.align_cost})" if r.align_cost is not None else ""))
    print(f"samples   : {r.samples} per direction")
    print(f"  mean surface dist : {r.mean} mm   ({r.mean_pct}% of bbox diag {r.bbox_diagonal} mm)")
    print(f"  rms               : {r.rms} mm")
    print(f"  median            : {r.median} mm")
    print(f"  95th percentile   : {r.p95} mm")
    print(f"  hausdorff (max)   : {r.hausdorff} mm   ({r.hausdorff_pct}% of bbox diag)")
    if r.volume_delta_pct is not None:
        print(f"  volume            : ref={r.reference_volume} other={r.other_volume} mm^3  (delta {r.volume_delta_pct}%)")
    else:
        print(f"  volume            : not watertight on both meshes; skipped")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("reference", type=Path, help="original / ground-truth STL")
    parser.add_argument("other", type=Path, help="reconstructed / derived STL to score")
    parser.add_argument("--samples", type=int, default=100000, help="surface samples per direction")
    parser.add_argument(
        "--align",
        choices=("none", "centroid", "icp"),
        default="none",
        help="register `other` onto `reference` first (default none: assume shared frame)",
    )
    parser.add_argument("--seed", type=int, default=0, help="sampling seed for reproducibility")
    parser.add_argument("--format", choices=("text", "json"), default="text")
    args = parser.parse_args(argv)

    result = compare(args.reference, args.other, samples=args.samples, align=args.align, seed=args.seed)
    if args.format == "json":
        print(json.dumps(asdict(result), indent=2))
    else:
        _print_text(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
