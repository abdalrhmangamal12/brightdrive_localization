#!/usr/bin/env python3
"""Initialization summary of localization runs, one row per recorded run bag (eval/run_init_tests.sh, run_live.sh).

Per run, from /localization/status, /localization/pose, /initialpose and /eval/ground_truth:
  - the start that reached TRACKING: seed_source, yaw_hint_source, the still verdict, init_v0 and init_sigma_v, next
    to the ground-truth speed at that search's sweep time (t_scan from init_last_search)
  - time to TRACKING: from the first IMU sample (status time_to_tracking_s) and from the /initialpose stamp
  - searches, failed confirmations, and every distinct search result (best and second error per inlier, the lead =
    second - best that init.margin is compared with, or the reason it failed)
  - accuracy after TRACKING: horizontal and heading error of /localization/pose against ground truth
  - a run that never reaches TRACKING: its last status message
Ground truth is used here only.

  python3 eval/init_report.py /bags/loc_runs/<run> [...]
"""
import math
import re
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/localization/status', '/localization/pose', '/initialpose', '/eval/ground_truth']


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    out = {t: [] for t in TOPICS}
    while r.has_next():
        topic, data, _ = r.read_next()
        out[topic].append(deserialize_message(data, get_message(types[topic])))
    return out


def summarize(bag):
    m = read(bag)
    status = []
    for s in m['/localization/status']:
        if s.status:
            kv = {v.key: v.value for v in s.status[0].values}
            status.append((stamp(s), s.status[0].level, s.status[0].message, kv))
    gt = np.array([(stamp(g), g.pose.pose.position.x, g.pose.pose.position.y, yaw_of(g.pose.pose.orientation),
                    math.hypot(g.twist.twist.linear.x, g.twist.twist.linear.y)) for g in m['/eval/ground_truth']])
    row = {'run': bag.rstrip('/').split('/')[-1], 'tracking': False}
    searches = []
    for _, _, _, kv in status:
        s = kv.get('init_last_search', '-')
        if s != '-' and (not searches or searches[-1] != s):
            searches.append(s)
    row['searches'] = searches
    # Why confirmations failed and what the node waited for, in order of appearance.
    row['init_states'] = []
    for _, _, _, kv in status:
        st = kv.get('init', '-')
        if ('confirmation failed' in st or 'waiting' in st) and st not in row['init_states']:
            row['init_states'].append(st)
        cf = kv.get('init_last_confirm_failure', '-')
        if cf != '-' and 'confirm failure: ' + cf not in row['init_states']:
            row['init_states'].append('confirm failure: ' + cf)
    if m['/initialpose']:
        row['t_initialpose'] = stamp(m['/initialpose'][0])
    first = next((x for x in status if x[3].get('lifecycle') == 'TRACKING'), None)
    if first is None:
        row['last_message'] = status[-1][2] if status else 'no status'
        return row
    t_track, _, _, kv = first
    row.update(tracking=True, t_track=t_track, kv=kv)
    ts = re.search(r't_scan ([0-9.]+)', kv.get('init_last_search', ''))
    if ts and len(gt):
        t_scan = float(ts.group(1))
        row['t_scan'] = t_scan
        row['gt_speed'] = gt[np.argmin(abs(gt[:, 0] - t_scan)), 4]
    # Accuracy after TRACKING.
    pose = [(stamp(p), p.pose.pose.position.x, p.pose.pose.position.y, yaw_of(p.pose.pose.orientation))
            for p in m['/localization/pose'] if stamp(p) >= t_track]
    if pose and len(gt):
        p = np.array(pose)
        p = p[(p[:, 0] >= gt[0, 0]) & (p[:, 0] <= gt[-1, 0])]
        gx, gy = np.interp(p[:, 0], gt[:, 0], gt[:, 1]), np.interp(p[:, 0], gt[:, 0], gt[:, 2])
        gyaw = np.interp(p[:, 0], gt[:, 0], np.unwrap(gt[:, 3]))
        h = np.hypot(p[:, 1] - gx, p[:, 2] - gy)
        dyaw = np.degrees(np.abs(np.angle(np.exp(1j * (p[:, 3] - gyaw)))))
        row.update(h_med=np.median(h), h_p95=np.percentile(h, 95), h_max=h.max(), yaw_p95=np.percentile(dyaw, 95),
                   h_first=h[0], n_pose=len(p), track_span=p[-1, 0] - p[0, 0])
    lost = [x for x in status if x[0] > t_track and x[3].get('lifecycle') != 'TRACKING']
    row['lost_after'] = len(lost)
    return row


def lead(s):
    e = re.search(r'e/inl ([0-9.]+|inf), second ([0-9.]+|inf)', s)
    return float(e.group(2)) - float(e.group(1)) if e else float('nan')


def main():
    rows = [summarize(b) for b in sys.argv[1:]]
    print('| run | TRACKING | seed | yaw hint | still | v0 / GT speed [m/s] | sigma_v | to TRACKING [s] '
          '(first IMU / initialpose) | searches / failed confirm | lead at the start | h p95 / max [m] | yaw p95 [deg] |')
    print('|' + '---|' * 12)
    for r in rows:
        if not r['tracking']:
            print(f"| {r['run']} | no | | | | | | | {len(r['searches'])} | | | |  last: {r['last_message']}")
            continue
        kv = r['kv']
        still = kv.get('still', '-').split(' ')[0].rstrip(':')
        from_ip = f" / {r['t_track'] - r['t_initialpose']:.2f}" if 't_initialpose' in r else ''
        print(f"| {r['run']} | yes | {kv.get('seed_source')} | {kv.get('yaw_hint_source')} | {still} | "
              f"{kv.get('init_v0')} / {r.get('gt_speed', float('nan')):.2f} | {kv.get('init_sigma_v')} | "
              f"{kv.get('time_to_tracking_s')}{from_ip} | {kv.get('init_searches')} / {kv.get('init_confirm_failures')} | "
              f"{lead(kv.get('init_last_search', '')):.3f} | {r.get('h_p95', float('nan')):.3f} / "
              f"{r.get('h_max', float('nan')):.3f} | {r.get('yaw_p95', float('nan')):.2f} |")
    for r in rows:
        print(f"\n{r['run']}: {len(r['searches'])} distinct search results"
              + (f", TRACKING at t {r['t_track']:.2f}, lost afterwards in {r['lost_after']} status messages, "
                 f"first-pose error {r.get('h_first', float('nan')):.3f} m, still: {r['kv'].get('still')}"
                 if r['tracking'] else ''))
        for s in r['searches']:
            print(f'  lead {lead(s):6.3f} | {s}')
        for s in r['init_states']:
            print(f'  init: {s}')


if __name__ == '__main__':
    main()
