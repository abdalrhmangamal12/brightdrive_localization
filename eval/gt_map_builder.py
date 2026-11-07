#!/usr/bin/env python3
"""Reference map from the raw LiDAR scans placed at ground-truth poses: the yardstick for a mapped point cloud.

Every /points_raw sweep is split by its per-point `time` field into the slices CARLA simulated (one world tick
each, 10 ms apart), each slice is moved to the map frame with the GT pose of that tick,

    p_map = T_map_base(t_stamp + time) * T_base_lidar * p_lidar

and everything is voxelised (centroid per cell). Range-cropped like the mapping pipeline so the car's own returns
are not in it. The result has the same sensor noise and the same traffic as a map built from this bag, so a
cloud-to-cloud distance against it measures pose error, not sensor error. Study-only: it uses ground truth.

Inside the mapping container (ROS sourced, rosbag2_py available):
  python3 eval/gt_map_builder.py <raw_bag_dir> <out.pcd> [--voxel 0.2] [--min-range 3] [--max-range 100]
                                 [--lidar-topic /points_raw] [--gt-topic /eval/ground_truth]
                                 [--sensors sim/config/sensors.yaml] [--start T] [--end T]
"""
import argparse
import math
import os
import time as wall
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointField

ROOT = Path(__file__).resolve().parents[1]
MERGE_EVERY = 250  # sweeps between two voxel merges; bounds the per-chunk memory

NP_TYPE = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2', PointField.UINT16: 'u2',
           PointField.INT32: 'i4', PointField.UINT32: 'u4', PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}


# ---------------------------------------------------------------- geometry
def rpy_deg_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), angles in degrees."""
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lidar_mount(sensors_yaml):
    with open(sensors_yaml) as f:
        cfg = yaml.safe_load(f)
    lidar = cfg.get('lidar')
    if lidar is None:
        lidar = next(v['lidar'] for v in cfg.values() if isinstance(v, dict) and 'lidar' in v)
    return rpy_deg_to_matrix(*lidar['rpy']), np.array(lidar['xyz'], dtype=float)


# ---------------------------------------------------------------- bag io
def open_reader(bag, topics):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in topics:
        if t not in types:
            raise SystemExit(f'{t} is not in {bag}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader, {t: get_message(types[t]) for t in topics}


def stamp_ms(header):
    return header.stamp.sec * 1000 + int(round(header.stamp.nanosec / 1e6))


def read_gt(bag, topic):
    """GT poses keyed by stamp in whole milliseconds (the sim tick is 10 ms, so the keys are exact)."""
    reader, types = open_reader(bag, [topic])
    poses = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, types[topic])
        p, q = m.pose.pose.position, m.pose.pose.orientation
        poses[stamp_ms(m.header)] = (quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]))
    return poses


def cloud_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(NP_TYPE[f.datatype])
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})


# ---------------------------------------------------------------- voxel grid
OFFSET = 1 << 20  # cells per axis before the origin, so negative coordinates still give a positive key
MASK = (1 << 21) - 1


def voxel_keys(p, voxel):
    c = np.floor(p / voxel).astype(np.int64) + OFFSET
    return (c[:, 0] & MASK) | ((c[:, 1] & MASK) << 21) | ((c[:, 2] & MASK) << 42)


def reduce_cells(keys, sums, counts):
    """Merge rows with the same key: sums (N, 4) are x y z intensity, counts (N,)."""
    uniq, inv = np.unique(keys, return_inverse=True)
    out = np.empty((len(uniq), 4))
    for c in range(4):
        out[:, c] = np.bincount(inv, weights=sums[:, c], minlength=len(uniq))
    n = np.bincount(inv, weights=counts, minlength=len(uniq))
    return uniq, out, n


def write_pcd(path, xyzi):
    rec = np.zeros(len(xyzi), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
    for i, name in enumerate(rec.dtype.names):
        rec[name] = xyzi[:, i]
    header = ('# .PCD v0.7 - Point Cloud Data file format\n# GT reference map: raw scans at ground-truth poses '
              '(eval/gt_map_builder.py). Study-only.\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
              % (len(rec), len(rec)))
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(rec.tobytes())


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--voxel', type=float, default=0.2, help='cell size [m], same as the mapping backend')
    ap.add_argument('--min-range', type=float, default=3.0, help='drop returns closer than this [m] (own car)')
    ap.add_argument('--max-range', type=float, default=100.0)
    ap.add_argument('--lidar-topic', default='/points_raw')
    ap.add_argument('--gt-topic', default='/eval/ground_truth')
    ap.add_argument('--sensors', default=str(ROOT / 'sim' / 'config' / 'sensors.yaml'))
    ap.add_argument('--start', type=float, default=None, help='first sweep stamp to use [s, sim time]')
    ap.add_argument('--end', type=float, default=None, help='last sweep stamp to use [s, sim time]')
    args = ap.parse_args()

    t0 = wall.time()
    R_bl, t_bl = lidar_mount(args.sensors)
    print(f'T_base_lidar: t = {t_bl.tolist()} m, R = identity' if np.allclose(R_bl, np.eye(3))
          else f'T_base_lidar: t = {t_bl.tolist()} m, R =\n{R_bl}')
    gt = read_gt(args.bag, args.gt_topic)
    gt_times = np.array(sorted(gt))
    print(f'{len(gt)} GT poses, {gt_times[0] / 1000:.3f}..{gt_times[-1] / 1000:.3f} s')

    reader, types = open_reader(args.bag, [args.lidar_topic])
    merged = None  # (keys, sums, counts) of everything so far
    chunk_keys, chunk_sums, chunk_counts = [], [], []
    n_sweeps = n_used = n_points = n_in_range = n_slices = n_slices_no_gt = 0
    dt = None
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)

    def merge_chunk():
        nonlocal merged, chunk_keys, chunk_sums, chunk_counts
        if not chunk_keys:
            return
        keys = np.concatenate(chunk_keys)
        sums = np.concatenate(chunk_sums)
        counts = np.concatenate(chunk_counts)
        if merged is not None:
            keys = np.concatenate([merged[0], keys])
            sums = np.concatenate([merged[1], sums])
            counts = np.concatenate([merged[2], counts])
        merged = reduce_cells(keys, sums, counts)
        chunk_keys, chunk_sums, chunk_counts = [], [], []

    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, types[args.lidar_topic])
        n_sweeps += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (args.start is not None and stamp < args.start) or (args.end is not None and stamp > args.end):
            continue
        if dt is None:
            dt = cloud_dtype(msg)
            print(f'point layout: {dt.names}, {msg.point_step} B/point')
        pts = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)
        rng = np.linalg.norm(xyz, axis=1)
        keep = (rng >= args.min_range) & (rng <= args.max_range) & np.isfinite(rng)
        n_points += len(xyz)
        n_in_range += int(keep.sum())
        xyz = xyz[keep]
        inten = pts['intensity'][keep].astype(np.float64) if 'intensity' in dt.names else np.zeros(len(xyz))
        t_rel = pts['time'][keep].astype(np.float64) if 'time' in dt.names else np.zeros(len(xyz))
        base_ms = stamp_ms(msg.header)
        p_base = xyz @ R_bl.T + t_bl
        p_map = np.empty_like(p_base)
        valid = np.zeros(len(p_base), dtype=bool)
        # One GT pose per slice: the points of a slice were simulated in the same world tick.
        for t_slice in np.unique(t_rel):
            sel = t_rel == t_slice
            key = base_ms + int(round(t_slice * 1000))
            n_slices += 1
            pose = gt.get(key)
            if pose is None:  # nearest tick within 5 ms, else the slice is dropped
                i = np.searchsorted(gt_times, key)
                cands = gt_times[max(0, i - 1):i + 1]
                if len(cands) and abs(int(cands[np.argmin(np.abs(cands - key))]) - key) <= 5:
                    pose = gt[int(cands[np.argmin(np.abs(cands - key))])]
            if pose is None:
                n_slices_no_gt += 1
                continue
            R_mb, t_mb = pose
            p_map[sel] = p_base[sel] @ R_mb.T + t_mb
            valid[sel] = True
        p_map, inten = p_map[valid], inten[valid]
        if len(p_map) == 0:
            continue
        n_used += 1
        lo = np.minimum(lo, p_map.min(0))
        hi = np.maximum(hi, p_map.max(0))
        keys, sums, counts = reduce_cells(voxel_keys(p_map, args.voxel),
                                          np.column_stack([p_map, inten]), np.ones(len(p_map)))
        chunk_keys.append(keys)
        chunk_sums.append(sums)
        chunk_counts.append(counts)
        if len(chunk_keys) >= MERGE_EVERY:
            merge_chunk()
            print(f'  {n_sweeps} sweeps, {len(merged[0])} voxels, {wall.time() - t0:.0f} s', flush=True)
    merge_chunk()
    if merged is None:
        raise SystemExit('no sweep could be placed: check the topics and the time window')

    keys, sums, counts = merged
    xyzi = sums / counts[:, None]
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_pcd(args.out, xyzi)
    print(f'sweeps {n_sweeps} used {n_used}; points {n_points} in range {n_in_range} '
          f'({100.0 * n_in_range / max(1, n_points):.1f} %); slices {n_slices}, without GT {n_slices_no_gt}')
    print(f'{len(xyzi)} voxels at {args.voxel} m -> {args.out}; extent x {lo[0]:.1f}..{hi[0]:.1f} '
          f'y {lo[1]:.1f}..{hi[1]:.1f} z {lo[2]:.1f}..{hi[2]:.1f} m; {wall.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
