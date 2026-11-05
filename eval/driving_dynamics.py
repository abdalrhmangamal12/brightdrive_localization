#!/usr/bin/env python3
"""Driving dynamics of a recorded run, from ground truth only: lateral acceleration and yaw rate against route
metres (speed for context), p95 / p99 / max, and the peaks (the corners). Nothing here is a localization input.

Reads /eval/ground_truth (nav_msgs/Odometry, pose in map, twist in base_link) from a bag.
  route metres    the ground-truth position projected onto the route (sim/scenarios/routes/<route>.yaml), so the
                  axis is sim.yaml's metres (speed profile, lane changes, corners). The path length driven is 2-3 %
                  shorter (the car cuts the corners): --route none uses it instead, plus --route-offset.
  speed           |v| of the body twist
  lateral accel   a_body = R^T d(v_map)/dt, y component, with v_map = R v_body using the yaw only (the road is nearly
                  flat); centred difference over --window seconds (0.10 s = 10 ticks by default)
  yaw rate        twist.angular.z of the ground truth
Statistics over the moving part (speed > 1 m/s): p95, p99, max of |lateral accel| and |yaw rate|. Peaks: local
maxima of |lateral accel| above --peak-threshold, at least --peak-separation route metres apart, each with its
speed and yaw rate (on a corner speed x yaw rate ~ lateral accel, speed / yaw rate = the radius).
--stretch A B: the same statistics for route metres A..B only (e.g. the lane-change straight).

Inside the sim container (rosbag2_py and matplotlib are there; the bags are under /bags):
  docker compose run --rm --no-deps -T sim python3 eval/driving_dynamics.py /bags/<bag> --out eval/plots/<name>.png
"""
import argparse
import math
import os
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')   # the container user has no home
import matplotlib  # noqa: E402

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

# Colours: one series per panel (blue), reference lines and axes muted, text in ink.
BLUE, MUTED, GRID, AXIS, INK, INK2, SURFACE = '#2a78d6', '#898781', '#e1e0d9', '#c3c2b7', '#0b0b0b', '#52514e', '#fcfcfb'


def read_ground_truth(bag):
    """(t, x, y, yaw, vx, vy, wz) arrays from /eval/ground_truth."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/eval/ground_truth':
            m = deserialize_message(data, Odometry)
            q, tw = m.pose.pose.orientation, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                         m.pose.pose.position.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z))
    if len(rows) < 3:
        raise SystemExit(f'{bag}: {len(rows)} /eval/ground_truth messages, nothing to analyse')
    return np.array(rows).T


ROOT = Path(__file__).resolve().parents[1]


def route_metres(x, y, route_file):
    """Metres along the route for each position: the nearest point of the route polyline, searched near the
    previous match (the loop closes, so the start would otherwise also match the end), laps unwrapped."""
    route = yaml.safe_load(open(route_file))
    pts = np.array([w[:2] for w in route['waypoints']], dtype=float)
    if route.get('closed', True):
        pts = np.vstack([pts, pts[:1]])
    seg = np.diff(pts, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    n = len(seg)

    def nearest(px, py, idx):
        d = pts[idx] + 0  # start points of the candidate segments
        t = np.clip(((px - d[:, 0]) * seg[idx, 0] + (py - d[:, 1]) * seg[idx, 1]) / seg_len[idx] ** 2, 0, 1)
        dist = np.hypot(px - (d[:, 0] + t * seg[idx, 0]), py - (d[:, 1] + t * seg[idx, 1]))
        # Ties go to the earlier metre: at the start the end of the closing segment is as near as segment 0.
        k = int((dist + 1e-6 * cum[idx]).argmin())
        return idx[k], cum[idx[k]] + t[k] * seg_len[idx[k]], dist[k]

    out, offs = np.empty(len(x)), np.empty(len(x))
    j, laps = nearest(x[0], y[0], np.arange(n))[0], 0
    for i in range(len(x)):
        # Candidates: a few segments behind to ~20 ahead of the previous match (wrapping at the loop's end).
        idx = (np.arange(j - 3, j + 20) % n) if route.get('closed', True) else np.clip(np.arange(j - 3, j + 20), 0, n - 1)
        jn, m, offs[i] = nearest(x[i], y[i], idx)
        if jn < j - 3:          # wrapped past the end of the loop
            laps += 1
        j = jn
        out[i] = m + laps * cum[-1]
    return out, offs


def dynamics(t, x, y, yaw, vx, vy, wz, window_s):
    """path length driven, speed, lateral acceleration (body y), longitudinal acceleration (body x)."""
    route = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    speed = np.hypot(vx, vy)
    c, s = np.cos(yaw), np.sin(yaw)
    vmx, vmy = c * vx - s * vy, s * vx + c * vy
    dt_med = np.median(np.diff(t))
    half = max(int(round(window_s / dt_med / 2)), 1)
    k = np.arange(len(t))
    i0, i1 = np.clip(k - half, 0, len(t) - 1), np.clip(k + half, 0, len(t) - 1)
    dt = t[i1] - t[i0]
    amx, amy = (vmx[i1] - vmx[i0]) / dt, (vmy[i1] - vmy[i0]) / dt
    return route, speed, -s * amx + c * amy, c * amx + s * amy


def peaks(route, sig, threshold, separation_m, n_max=20):
    """Indices of the largest |sig| values, each at least separation_m route metres from the ones kept before."""
    out = []
    for i in np.argsort(-np.abs(sig)):
        if abs(sig[i]) < threshold or len(out) == n_max:
            break
        if all(abs(route[i] - route[j]) >= separation_m for j in out):
            out.append(i)
    return sorted(out, key=lambda i: route[i])


def stats(name, sig, unit):
    a = np.abs(sig)
    print(f'  {name:22s} p95 {np.percentile(a, 95):7.2f}   p99 {np.percentile(a, 99):7.2f}   max {a.max():7.2f} {unit}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag directory with /eval/ground_truth')
    ap.add_argument('--out', help='PNG to write (default: no plot)')
    ap.add_argument('--title', help='plot title (default: the bag name)')
    ap.add_argument('--window', type=float, default=0.10, help='differentiation window [s] (0.10)')
    ap.add_argument('--route', default=str(ROOT / 'sim/scenarios/routes/main.yaml'),
                    help='route file to project on (sim/scenarios/routes/main.yaml); "none" = path length driven')
    ap.add_argument('--route-offset', type=float, default=0.0, help='with --route none: route metre of the first pose')
    ap.add_argument('--peak-threshold', type=float, default=4.0, help='|lateral accel| a peak must reach [m/s^2] (4)')
    ap.add_argument('--peak-separation', type=float, default=20.0, help='minimum distance between peaks [m] (20)')
    ap.add_argument('--stretch', type=float, nargs=2, metavar=('A', 'B'), help='also report route metres A..B')
    a = ap.parse_args()

    t, x, y, yaw, vx, vy, wz = read_ground_truth(a.bag)
    driven, speed, alat, along = dynamics(t, x, y, yaw, vx, vy, wz, a.window)
    if a.route.lower() == 'none':
        route, axis_label = driven + a.route_offset, 'metres driven (ground-truth path length)'
    else:
        route, off = route_metres(x, y, a.route)
        axis_label = 'route metres (ground truth projected on the route)'
        print(f'route {Path(a.route).name}: ground truth projected on it, offset from the route line max {off.max():.2f} m; '
              f'path length driven {driven[-1]:.1f} m = route metres {route[0]:.1f} .. {route[-1]:.1f}')
    wz_deg = np.degrees(wz)
    moving = speed > 1.0
    print(f'{a.bag}: {len(t)} ground-truth msgs, {t[-1] - t[0]:.1f} s, {driven[-1]:.1f} m driven, '
          f'speed max {speed.max():.2f} m/s; window {a.window:.2f} s; statistics over {moving.sum()} moving samples '
          f'(speed > 1 m/s)')
    stats('|lateral accel|', alat[moving], 'm/s^2')
    stats('|yaw rate|', wz_deg[moving], 'deg/s')
    stats('|longitudinal accel|', along[moving], 'm/s^2')
    pk = peaks(route, alat, a.peak_threshold, a.peak_separation)
    print(f'  peaks of |lateral accel| >= {a.peak_threshold:g} m/s^2, >= {a.peak_separation:g} m apart ({len(pk)}):')
    for i in pk:
        w = abs(wz[i])
        print(f'    route {route[i]:7.1f} m  t {t[i] - t[0]:6.2f} s  a_lat {alat[i]:+6.2f} m/s^2  speed {speed[i]:5.2f} m/s'
              f'  yaw rate {wz_deg[i]:+6.1f} deg/s  speed x yaw rate {speed[i] * w:5.2f}  radius {speed[i] / max(w, 1e-3):5.1f} m')
    if a.stretch:
        lo, hi = a.stretch
        m = moving & (route >= lo) & (route <= hi)
        print(f'  stretch {lo:g}-{hi:g} m ({m.sum()} samples):')
        if m.any():
            stats('|lateral accel|', alat[m], 'm/s^2')
            stats('|yaw rate|', wz_deg[m], 'deg/s')
            j = np.where(m)[0][np.abs(alat[m]).argmax()]
            print(f'    |lateral accel| max at route {route[j]:.1f} m, speed {speed[j]:.2f} m/s, yaw rate {wz_deg[j]:+.1f} deg/s;'
                  f' |v_y| max {np.abs(vy[m]).max():.2f} m/s')

    if not a.out:
        return
    fig, axes = plt.subplots(3, 1, figsize=(12, 7.5), dpi=110, sharex=True, facecolor=SURFACE)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.93, bottom=0.08, hspace=0.18)
    panels = ((speed, 'speed [m/s]', None), (alat, 'lateral acceleration [m/s²]', 8.0), (wz_deg, 'yaw rate [deg/s]', None))
    for ax, (sig, label, ref) in zip(axes, panels):
        ax.set_facecolor(SURFACE)
        ax.plot(route, sig, color=BLUE, linewidth=1.4)
        if ref is not None:
            for v in (ref, -ref):
                ax.axhline(v, color=MUTED, linewidth=1.0, linestyle=(0, (4, 3)))
            ax.annotate(f'{ref:g} m/s²', (route[-1], ref), xytext=(-4, 3), textcoords='offset points', ha='right',
                        fontsize=8.5, color=INK2)
            # Peak labels; neighbours closer than 4 % of the route alternate between two rows so they do not overlap.
            row, last = 0, -1e9
            for i in pk:
                row = 1 - row if route[i] - last < 0.04 * (route[-1] - route[0]) else 0
                last = route[i]
                ax.plot(route[i], alat[i], 'o', color=BLUE, markersize=5.5, markeredgecolor=SURFACE, markeredgewidth=1.2)
                dy = -(12 + 11 * row) if alat[i] < 0 else 6 + 11 * row
                ax.annotate(f'{route[i]:.0f} m', (route[i], alat[i]), xytext=(0, dy), textcoords='offset points',
                            ha='center', fontsize=8, color=INK2)
        ax.set_ylabel(label, color=INK, fontsize=9.5)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.tick_params(colors=INK2, labelsize=8.5)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color(AXIS)
    axes[-1].set_xlabel(axis_label, color=INK, fontsize=9.5)
    title = a.title or Path(a.bag).name
    sub = (f'ground truth, {a.window:.2f} s window; moving part: |a_lat| p95 {np.percentile(np.abs(alat[moving]), 95):.1f} / '
           f'p99 {np.percentile(np.abs(alat[moving]), 99):.1f} / max {np.abs(alat[moving]).max():.1f} m/s², '
           f'|yaw rate| p95 {np.percentile(np.abs(wz_deg[moving]), 95):.0f} / p99 {np.percentile(np.abs(wz_deg[moving]), 99):.0f} / '
           f'max {np.abs(wz_deg[moving]).max():.0f} deg/s')
    fig.suptitle(title, color=INK, fontsize=12, x=0.07, ha='left', y=0.985)
    fig.text(0.07, 0.945, sub, color=INK2, fontsize=9)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, facecolor=SURFACE)
    print(f'  plot: {a.out}')


if __name__ == '__main__':
    main()
