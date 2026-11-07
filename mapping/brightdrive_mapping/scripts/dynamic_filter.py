#!/usr/bin/env python3
"""Remove dynamic objects from a mapping run with DUFOMap.

  ros2 run brightdrive_mapping dynamic_filter.py <save_dir>      # e.g. /maps; or run_dynamic_filter:=true in the launch

The one argument is the mapping output folder. map.yaml there gives the map's name (map.pcd), frame and voxel
size; keyframes/NNNNNN.pcd (written with save_keyframes:=true) is the input: every keyframe in the map frame with
the sensor pose in the VIEWPOINT line (the DUFOMap / DynamicMap_Benchmark unified format). DUFOMap casts the rays
of every frame from its viewpoint and marks the cells they pass through as seen-free; a point whose cell was seen
free at another time is dynamic ("DUFOMap: Efficient Dynamic Awareness Mapping", Duberg, Zhang, Jia, Jensfelt,
RA-L 2024). The parameters are the ones DUFOMap's own main.py uses (resolution 0.1 m, d_s 0.2 m, d_p 2) and
nothing is tuned here; the keyframe clouds are already range-cropped by the backend, so main.py's 50 m cut is not
repeated.

Output, next to map.yaml: <name>_static.pcd (the points labelled static, voxelised like the backend's map),
<name>_static.yaml (map.yaml with map.pcd pointing at the static file plus a dynamic_filter block),
<name>_dynamic.pcd (what was removed), <name>_dynamic_bev.png (top-down, removed points in red), and the counts.
The mapping container has dufomap (docker/Dockerfile); on a host it is `pip install dufomap`.
"""
import argparse
import glob
import os
import tempfile
import time

import numpy as np
import yaml
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


def write_static_yaml(save_dir, map_name, removed_fraction, voxel):
    """<map>_static.yaml = <save_dir>/map.yaml with map.pcd pointing at the static map, plus a dynamic_filter
    block. A text copy, so the comments and the order of the backend's file survive. None when map.yaml is absent."""
    src = os.path.join(save_dir, 'map.yaml')
    dst = os.path.join(save_dir, f'{map_name}_static.yaml')
    if not os.path.exists(src):
        print(f'no {src}: {dst} not written')
        return None
    out, replaced = [], False
    with open(src) as f:
        for line in f.read().splitlines():
            if line.startswith('  pcd: ') and not replaced:
                comment = line.partition('#')[2]
                line = f'  pcd: {map_name}_static.pcd'.ljust(35) + (f'#{comment}' if comment else '')
                replaced = True
            out.append(line)
    if not replaced:
        raise SystemExit(f'{src}: no "  pcd:" line in the map block')
    out += ['',
            'dynamic_filter:                    # scripts/dynamic_filter.py',
            '  method: DUFOMap',
            f'  params: {{resolution_m: {RESOLUTION_M}, d_s_m: {D_S_M}, d_p: {D_P}}}',
            f'  removed_fraction: {removed_fraction:.4f}         # {voxel} m voxels of the input that are gone',
            f'  input: {map_name}.pcd']
    with open(dst, 'w') as f:
        f.write('\n'.join(out) + '\n')
    return dst


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
    """Top-down picture; False when matplotlib is not installed (the maps and the yaml do not depend on it)."""
    os.environ.setdefault('MPLCONFIGDIR', tempfile.mkdtemp(prefix='mpl-'))  # the container's HOME is not writable
    try:
        import matplotlib
    except ImportError:
        print(f'matplotlib not installed: no {path}')
        return False
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
    return True


def read_map_yaml(save_dir):
    """(name, frame, georeferenced, voxel) from <save_dir>/map.yaml, with clear messages when it is not there."""
    path = os.path.join(save_dir, 'map.yaml')
    if not os.path.isfile(path):
        raise SystemExit(f'{path} not found: {save_dir} is not a mapping output folder')
    with open(path) as f:
        desc = yaml.safe_load(f)
    try:
        m = desc['map']
        pcd, frame, georeferenced, voxel = m['pcd'], m['frame'], bool(m['georeferenced']), float(m['resolution_m'])
    except (KeyError, TypeError):
        raise SystemExit(f'{path}: no map block with pcd, frame, georeferenced, resolution_m '
                         f'(the file predates the map.yaml contract; re-run mapping)')
    name = pcd[:-4] if pcd.endswith('.pcd') else pcd
    return name, frame, georeferenced, voxel


def main():
    ap = argparse.ArgumentParser(
        usage='ros2 run brightdrive_mapping dynamic_filter.py <save_dir>',
        description='Remove dynamic objects from a mapping run: DUFOMap over <save_dir>/keyframes/, '
                    '<name>_static.pcd and <name>_static.yaml next to map.yaml. '
                    'Everything else (map name, frame, voxel size) comes from <save_dir>/map.yaml.')
    ap.add_argument('save_dir', help='the mapping output folder (save_dir of the backend), e.g. /maps')
    ap.add_argument('--threads', type=int, default=0, help='DUFOMap threads, 0 = all')
    ap.add_argument('--no-image', action='store_true', help='skip <name>_dynamic_bev.png')
    args = ap.parse_args()
    save_dir = os.path.abspath(args.save_dir)
    name, frame, georeferenced, voxel = read_map_yaml(save_dir)
    kf_dir = os.path.join(save_dir, 'keyframes')
    files = sorted(glob.glob(os.path.join(kf_dir, '*.pcd')))
    if not files:
        raise SystemExit(f'{kf_dir} is missing or empty: re-run mapping with save_keyframes:=true')
    print(f'{save_dir}: map {name}.pcd in frame {frame}'
          + ('' if georeferenced else ' (NOT georeferenced: the static map will not be either)')
          + f', voxel {voxel} m, {len(files)} keyframes')

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

    full_vox = voxel_centroids(np.vstack([static, dynamic]), voxel)
    static_vox = voxel_centroids(static, voxel)
    dynamic_vox = voxel_centroids(dynamic, voxel) if len(dynamic) else dynamic
    static_path = os.path.join(save_dir, f'{name}_static.pcd')
    dynamic_path = os.path.join(save_dir, f'{name}_dynamic.pcd')
    write_pcd(static_path, static_vox, 'static map: DUFOMap-labelled static points (brightdrive_mapping/scripts/dynamic_filter.py)')
    if len(dynamic_vox):
        write_pcd(dynamic_path, dynamic_vox, 'points DUFOMap removed as dynamic (brightdrive_mapping/scripts/dynamic_filter.py)')
    removed_fraction = (len(full_vox) - len(static_vox)) / max(1, len(full_vox))
    yaml_path = write_static_yaml(save_dir, name, removed_fraction, voxel)

    print(f'{len(files)} keyframes, {n_all:,} points; DUFOMap(resolution {RESOLUTION_M}, d_s {D_S_M}, d_p {D_P}) '
          f'integrated in {t_int:.0f} s, labelled in {time.time() - t0 - t_int:.0f} s')
    print(f'removed {len(dynamic):,} points ({100.0 * len(dynamic) / max(1, n_all):.2f} %) '
          f'-> static {len(static):,}')
    print(f'at {voxel} m voxels: full map {len(full_vox):,}, static {len(static_vox):,} '
          f'({100.0 * (len(full_vox) - len(static_vox)) / max(1, len(full_vox)):.2f} % fewer), '
          f'removed {len(dynamic_vox):,}')
    print(f'wrote {static_path}' + (f', {yaml_path}' if yaml_path else '')
          + (f', {dynamic_path}' if len(dynamic_vox) else ''), flush=True)
    # Last, so a missing matplotlib costs the picture and nothing else.
    if not args.no_image:
        image_path = os.path.join(save_dir, f'{name}_dynamic_bev.png')
        if bev_image(image_path, static_vox, dynamic_vox):
            print(f'wrote {image_path}')


if __name__ == '__main__':
    main()
