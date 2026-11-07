#!/usr/bin/env python3
"""Export /eval/ground_truth to a TUM trajectory in the IMU frame, the evo reference for the mapping trajectories.

Ground truth is base_link in map; the mapping trajectories are the IMU body (the front-end estimates
world -> IMU). Every GT pose is therefore moved to the IMU before it is written:

    T_map_imu = T_map_base * T_base_imu

with T_base_imu from the IMU mount in sim/config/sensors.yaml (xyz in m, rpy in degrees, ZYX). It is the identity
in the current setup, and the transform used is printed so the file can be trusted either way. t is the header
stamp (sim time); rows are sorted by time.

Inside the mapping container (ROS sourced, rosbag2_py available):
  python3 eval/gt_to_tum.py <bag_dir> <out.txt> [--topic /eval/ground_truth] [--sensors sim/config/sensors.yaml]
"""
import argparse
import math
import os
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[1]


def rpy_deg_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), angles in degrees."""
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_quat(R):
    """x, y, z, w."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                         (R[1, 0] - R[0, 1]) / (4 * w), w])
    # 180-degree case, not expected for a sensor mount but kept correct.
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
    q = np.zeros(4)
    q[i], q[j], q[k], q[3] = s / 4.0, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s, (R[k, j] - R[j, k]) / s
    return q


def quat_mul(a, b):
    """Hamilton product, both x y z w."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def quat_rotate(q, v):
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R @ v


def imu_mount(sensors_yaml):
    """t_base_imu [m] and q_base_imu (x y z w) from the sensors file; the imu block may sit one level down."""
    with open(sensors_yaml) as f:
        cfg = yaml.safe_load(f)
    imu = cfg.get('imu')
    if imu is None:
        imu = next(v['imu'] for v in cfg.values() if isinstance(v, dict) and 'imu' in v)
    t = np.array(imu['xyz'], dtype=float)
    R = rpy_deg_to_matrix(*imu['rpy'])
    return t, matrix_to_quat(R)


def read_poses(bag, topic):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f'{topic} is not in {bag}; eval/gt_publisher.py records it')
    msg_type = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, msg_type)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])))
    rows.sort(key=lambda r: r[0])
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--topic', default='/eval/ground_truth')
    ap.add_argument('--sensors', default=str(ROOT / 'sim' / 'config' / 'sensors.yaml'))
    args = ap.parse_args()

    t_bi, q_bi = imu_mount(args.sensors)
    print(f'T_base_imu from {args.sensors}: t = {t_bi.tolist()} m, q (x y z w) = {np.round(q_bi, 9).tolist()}'
          + ('  (identity)' if np.allclose(t_bi, 0) and np.allclose(q_bi, [0, 0, 0, 1]) else ''))

    rows = read_poses(args.bag, args.topic)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        for t, p_mb, q_mb in rows:
            p = p_mb + quat_rotate(q_mb, t_bi)  # T_map_imu = T_map_base * T_base_imu
            q = quat_mul(q_mb, q_bi)
            f.write('%.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n' % (t, p[0], p[1], p[2], q[0], q[1], q[2], q[3]))
    print(f'{len(rows)} poses from {args.topic} -> {args.out} (IMU frame); t {rows[0][0]:.3f}..{rows[-1][0]:.3f}')


if __name__ == '__main__':
    main()
