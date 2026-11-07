#!/usr/bin/env python3
"""Cloud-to-cloud distance of a map against the GT reference map (eval/gt_map_builder.py).

For every point of <map.pcd> the distance to its nearest neighbour in <reference.pcd>: mean, median, p95, max and
the share beyond 0.5 m. Both clouds are 0.2 m voxel centroids, so two identical surfaces bottom out near 0.1 m, not
at zero; the reference against itself prints zeros. The reverse direction (reference -> map) is printed too: it
says how much of the reference the map covers, which the forward distance cannot.

  eval/map_quality.py <map.pcd> <reference.pcd> [--max-dist 5.0]
"""
import argparse
import re

import numpy as np
from scipy.spatial import cKDTree

PCD_TYPES = {('F', 4): 'f4', ('F', 8): 'f8', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4',
             ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4'}


def read_pcd_xyz(path):
    """x y z of a binary PCD as float64 (ascii PCDs are not handled)."""
    header = {}
    with open(path, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', 'replace')
            if not line:
                raise SystemExit(f'{path}: no DATA line')
            if line.startswith('#'):
                continue
            key, _, value = line.strip().partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        if header['DATA'][0] != 'binary':
            raise SystemExit(f'{path}: only DATA binary is handled, got {header["DATA"][0]}')
        names = header['FIELDS']
        formats = [PCD_TYPES[(t, int(s))] for t, s in zip(header['TYPE'], header['SIZE'])]
        counts = [int(c) for c in header.get('COUNT', ['1'] * len(names))]
        dtype = np.dtype([(n, fmt, c) if c > 1 else (n, fmt) for n, fmt, c in zip(names, formats, counts)])
        n = int(header['POINTS'][0])
        rec = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
    return np.column_stack([rec['x'], rec['y'], rec['z']]).astype(np.float64)


def nn_stats(src, dst, max_dist):
    d, _ = cKDTree(dst).query(src, k=1, distance_upper_bound=max_dist, workers=-1)
    far = ~np.isfinite(d)
    d = np.where(far, max_dist, d)
    return {'n': len(d), 'mean': d.mean(), 'median': np.median(d), 'p95': np.percentile(d, 95),
            'max': d.max(), 'over_0.5m': 100.0 * (d > 0.5).mean(), 'capped': 100.0 * far.mean()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map')
    ap.add_argument('reference')
    ap.add_argument('--max-dist', type=float, default=5.0, help='distances beyond this are capped [m]')
    args = ap.parse_args()
    m = read_pcd_xyz(args.map)
    r = read_pcd_xyz(args.reference)
    fwd = nn_stats(m, r, args.max_dist)
    rev = nn_stats(r, m, args.max_dist)
    print(f'map {args.map}: {len(m):,} points; reference {args.reference}: {len(r):,} points')
    print(f'{"direction":<20}{"mean":>8}{"median":>8}{"p95":>8}{"max":>8}{"> 0.5 m":>9}{"capped":>8}')
    for name, s in (('map -> reference', fwd), ('reference -> map', rev)):
        print(f'{name:<20}{s["mean"]:>8.3f}{s["median"]:>8.3f}{s["p95"]:>8.3f}{s["max"]:>8.3f}'
              f'{s["over_0.5m"]:>8.2f}%{s["capped"]:>7.2f}%')


if __name__ == '__main__':
    main()
