#!/usr/bin/env python3
"""Remove dynamic objects from the keyframe clouds with DUFOMap and write the static map.

Input: <save_dir>/keyframes/NNNNNN.pcd, written by the backend with save_keyframes: every keyframe in the map
frame with the sensor pose in the VIEWPOINT line (the DUFOMap / DynamicMap_Benchmark unified format). DUFOMap
casts the rays of every frame from its viewpoint and marks the cells they pass through as seen-free; a point whose
cell was seen free at another time is dynamic ("DUFOMap: Efficient Dynamic Awareness Mapping", Duberg, Zhang, Jia,
Jensfelt, RA-L 2024). The parameters are the ones DUFOMap's own main.py uses (resolution 0.1 m, d_s 0.2 m, d_p 2)
and nothing is tuned here; the keyframe clouds are already range-cropped by the backend, so main.py's 50 m cut is
not repeated.

Output in <save_dir>: <map>_static.pcd (the points labelled static, voxelised like the backend's map),
<map>_dynamic.pcd (what was removed), <map>_dynamic_bev.png (top-down, removed points in red), and the counts.

  pip install dufomap                       # host python; the mapping container has no dufomap
  mapping/brightdrive_mapping/scripts/dynamic_filter.py <save_dir> [--map-name town03] [--voxel 0.2] [--threads 0] [--no-image]
"""
import argparse
import glob
import os
import time

import numpy as np
from dufomap import dufomap

# DUFOMap main.py: dufomap(0.1, 0.2, 2) "resolution, d_s, d_p same with paper".
RESOLUTION_M = 0.1
D_S_M = 0.2
D_P = 2

OFFSET = 1 << 20  # voxel key layout of the backend: 21 bits per axis, origin in the middle
MASK = (1 << 21) - 1

PCD_TYPES = {('F', 4): 'f4', ('F', 8): 'f8', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4',
             ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4'}


def read_pcd(path):
    """Binary PCD -> (x y z intensity as float32 (N, 4), VIEWPOINT as [x y z qw qx qy qz]).
    Own reader: dufomap's bundled one uses np.fromstring, which NumPy 2 removed."""
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
            raise SystemExit(f'{path}: only DATA binary is handled')
        names = header['FIELDS']
        formats = [PCD_TYPES[(t, int(s))] for t, s in zip(header['TYPE'], header['SIZE'])]
        counts = [int(c) for c in header.get('COUNT', ['1'] * len(names))]
        dtype = np.dtype([(n, fmt, c) if c > 1 else (n, fmt) for n, fmt, c in zip(names, formats, counts)])
        n = int(header['POINTS'][0])
        rec = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
    xyzi = np.zeros((n, 4), dtype=np.float32)
    for i, axis in enumerate('xyz'):
        xyzi[:, i] = rec[axis]
    if 'intensity' in names:
        xyzi[:, 3] = rec['intensity']
    viewpoint = [float(v) for v in header.get('VIEWPOINT', '0 0 0 1 0 0 0'.split())]
    return xyzi, viewpoint


def write_pcd(path, xyzi, comment):
    rec = np.zeros(len(xyzi), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
    for i, name in enumerate(rec.dtype.names):
        rec[name] = xyzi[:, i]
    header = ('# .PCD v0.7 - Point Cloud Data file format\n# %s\nVERSION 0.7\nFIELDS x y z intensity\n'
              'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              'POINTS %d\nDATA binary\n' % (comment, len(rec), len(rec)))
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(rec.tobytes())


def voxel_centroids(xyzi, voxel):
    """Centroid per cell, same grid as the backend's saved map."""
    c = np.floor(xyzi[:, :3] / voxel).astype(np.int64) + OFFSET
    keys = (c[:, 0] & MASK) | ((c[:, 1] & MASK) << 21) | ((c[:, 2] & MASK) << 42)
    uniq, inv = np.unique(keys, return_inverse=True)
    out = np.empty((len(uniq), 4), dtype=np.float32)
    n = np.bincount(inv, minlength=len(uniq))
    for col in range(4):
        out[:, col] = np.bincount(inv, weights=xyzi[:, col], minlength=len(uniq)) / n
    return out


def bev_image(path, static, dynamic):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cell = 0.5
    allp = np.vstack([static[:, :2], dynamic[:, :2]])
    lo, hi = allp.min(0) - 5, allp.max(0) + 5
    bins = [np.arange(lo[0], hi[0] + cell, cell), np.arange(lo[1], hi[1] + cell, cell)]
    h, _, _ = np.histogram2d(static[:, 0], static[:, 1], bins=bins)
    fig, ax = plt.subplots(figsize=(12, 12 * (hi[1] - lo[1]) / (hi[0] - lo[0])))
    ax.imshow(np.log1p(h).T, origin='lower', extent=[lo[0], hi[0], lo[1], hi[1]], cmap='Greys', vmax=4)
    ax.scatter(dynamic[:, 0], dynamic[:, 1], s=0.3, c='red', linewidths=0, label=f'removed ({len(dynamic):,} pts)')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('static map (grey, log density) and the points DUFOMap removed (red)')
    ax.legend(loc='upper right', markerscale=20)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('save_dir')
    ap.add_argument('--map-name', default='town03')
    ap.add_argument('--voxel', type=float, default=0.2, help='cell size of the written maps [m], as the backend')
    ap.add_argument('--threads', type=int, default=0, help='DUFOMap threads, 0 = all')
    ap.add_argument('--no-image', action='store_true')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.save_dir, 'keyframes', '*.pcd')))
    if not files:
        raise SystemExit(f'no keyframes in {args.save_dir}/keyframes: run the backend with save_keyframes: true')
    t0 = time.time()
    mymap = dufomap(RESOLUTION_M, D_S_M, D_P, num_threads=args.threads)
    frames = []
    for i, f in enumerate(files):
        xyzi, pose = read_pcd(f)  # pose = VIEWPOINT, the [x y z qw qx qy qz] DUFOMap wants
        frames.append((xyzi, pose))
        mymap.run(xyzi[:, :3], pose, cloud_transform=False)  # points are already in the map frame
        mymap.oncePropagateCluster(if_propagate=True, if_cluster=False)
        if (i + 1) % 500 == 0:
            print(f'  integrated {i + 1}/{len(files)} frames, {time.time() - t0:.0f} s', flush=True)
    t_int = time.time() - t0

    static, dynamic = [], []
    for xyzi, pose in frames:
        dyn = mymap.segment(xyzi[:, :3], pose, cloud_transform=False).astype(bool)
        static.append(xyzi[~dyn])
        dynamic.append(xyzi[dyn])
    static = np.vstack(static)
    dynamic = np.vstack(dynamic) if any(len(d) for d in dynamic) else np.zeros((0, 4), np.float32)
    n_all = len(static) + len(dynamic)

    full_vox = voxel_centroids(np.vstack([static, dynamic]), args.voxel)
    static_vox = voxel_centroids(static, args.voxel)
    dynamic_vox = voxel_centroids(dynamic, args.voxel) if len(dynamic) else dynamic
    static_path = os.path.join(args.save_dir, f'{args.map_name}_static.pcd')
    dynamic_path = os.path.join(args.save_dir, f'{args.map_name}_dynamic.pcd')
    write_pcd(static_path, static_vox, 'static map: DUFOMap-labelled static points (brightdrive_mapping/scripts/dynamic_filter.py)')
    if len(dynamic_vox):
        write_pcd(dynamic_path, dynamic_vox, 'points DUFOMap removed as dynamic (brightdrive_mapping/scripts/dynamic_filter.py)')
    if not args.no_image:
        bev_image(os.path.join(args.save_dir, f'{args.map_name}_dynamic_bev.png'), static_vox, dynamic_vox)

    print(f'{len(files)} keyframes, {n_all:,} points; DUFOMap(resolution {RESOLUTION_M}, d_s {D_S_M}, d_p {D_P}) '
          f'integrated in {t_int:.0f} s, labelled in {time.time() - t0 - t_int:.0f} s')
    print(f'removed {len(dynamic):,} points ({100.0 * len(dynamic) / max(1, n_all):.2f} %) '
          f'-> static {len(static):,}')
    print(f'at {args.voxel} m voxels: full map {len(full_vox):,}, static {len(static_vox):,} '
          f'({100.0 * (len(full_vox) - len(static_vox)) / max(1, len(full_vox)):.2f} % fewer), '
          f'removed {len(dynamic_vox):,}')
    print(f'wrote {static_path}' + (f', {dynamic_path}' if len(dynamic_vox) else ''))


if __name__ == '__main__':
    main()
