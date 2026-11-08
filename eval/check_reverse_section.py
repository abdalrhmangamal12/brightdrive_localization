#!/usr/bin/env python3
"""The reserved reverse section against the mapping drive, from ground truth (evaluation only).

The mapping requirement: a mapping drive over an urban route with overlapping passes, and at least one road section
reserved for reverse-direction localization that was not traversed in that direction during mapping. Reads
/eval/ground_truth (base_link in map) from the mapping bag and the reverse_reuse bag, and the map, and reports:

  mapping drive   path length, time, speed, laps (returns to the start point); how many times each position of the
                  first lap is driven (same-direction passes within 3 m over the whole drive); stretches with extra
                  passes on another lane of the same road (within 6 m, either direction).
  reserved        sim.yaml routes.<route>.reserved_reverse, a waypoint range of routes/<route>.yaml, and its length.
  reverse drive   start, end, length; per straight leg the distance to the nearest mapping sample and the heading
                  difference there.
  proof           same direction: reverse positions with a mapping sample within 3 m heading the same way (within
                  30 deg); must be 0 %. Map coverage: reverse positions with at least 25 map points on the road
                  surface within 1 m (|dz| < 0.5 m of the car origin; a full 1 m disc holds about 79 cells of
                  0.2 m); must be 100 %.

Coverage, not trajectory distance, shows that the reverse lanes are mapped: driving a road the other way means
driving its other lanes, one lane (about 3.5 m) to a median (10-14 m) away from where the mapping car drove, so no
reverse position needs to lie within 3 m of a mapping position although the LiDAR mapped those lanes from across.

Positions are resampled every 0.5 m of travelled path, over the moving part only (speed > 0.5 m/s). Writes
reverse_section.md, reverse_section.json and reverse_vs_mapping.png into --out; exit 0 when both proofs hold.
Inside the loc container (ROS sourced):
  python3 eval/check_reverse_section.py /bags/mapping_20261105_070252 /bags/reverse_reuse_20261107_153511 \\
      --map-yaml /maps/town03_static.yaml --out eval/results/mapping
"""
import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

os.environ.setdefault('MPLCONFIGDIR', tempfile.mkdtemp(prefix='mpl-'))
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import map_quality  # noqa: E402

SAMPLE_M = 0.5            # resampling step along the travelled path
MOVING_MPS = 0.5          # the drive starts and ends where the car moves faster than this
SAME_LANE_M = 3.0         # a mapping sample this close is on the same lane
SAME_ROAD_M = 6.0         # this close: another lane of the same road
SAME_DEG = 30.0           # heading difference below this: same direction
OPPOSITE_DEG = 150.0      # above this: opposite direction
NEW_PASS_GAP_M = 20.0     # neighbours more than this apart along the path belong to different passes
START_NEAR_M, START_FAR_M = 5.0, 50.0   # a lap ends when the car is back within 5 m of the start after 50 m away
STRETCH_MIN_M = 10.0      # shortest overlap stretch reported
LEG_TOL_DEG = 15.0        # a straight leg heads within this of east / north / west / south
LEG_MIN_M = 20.0          # shortest straight leg; same-direction legs closer than this are merged
ROAD_RADIUS_M, ROAD_DZ_M, ROAD_MIN_POINTS = 1.0, 0.5, 25

BLUE, ORANGE = '#2a78d6', '#eb6834'       # categorical slots 1 and 2 (validated pair)
INK, INK2, SURFACE, GRID = '#0b0b0b', '#52514e', '#fcfcfb', '#e6e5e1'


def read_ground_truth(bag, topic='/eval/ground_truth'):
    """t, x, y, z, yaw of base_link in map, sorted by stamp."""
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
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw))
    a = np.array(sorted(rows))
    return {k: a[:, i] for i, k in enumerate(('t', 'x', 'y', 'z', 'yaw'))}


def moving_part(g):
    """g cut to the first..last sample of motion, plus the per-step speed inside it."""
    v = np.hypot(np.diff(g['x']), np.diff(g['y'])) / np.maximum(np.diff(g['t']), 1e-6)
    idx = np.where(v > MOVING_MPS)[0]
    i0, i1 = idx[0], idx[-1] + 1
    return {k: g[k][i0:i1 + 1] for k in g}, v[i0:i1], g['t'][i0] - g['t'][0]


def resample(g):
    """Samples every SAMPLE_M of travelled path, with s (path metres)."""
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(g['x']), np.diff(g['y'])))])
    keep = np.searchsorted(s, np.arange(0.0, s[-1], SAMPLE_M))
    out = {k: g[k][keep] for k in g}
    out['s'] = s[keep]
    return out, s[-1]


def heading_diff_deg(a, b):
    """|a - b| wrapped to 0..180 degrees, a and b in radians."""
    return np.abs((np.degrees(a - b) + 180.0) % 360.0 - 180.0)


def lap_ends(r):
    """Indices of the returns to the start point (closest approach each time)."""
    d0 = np.hypot(r['x'] - r['x'][0], r['y'] - r['y'][0])
    ends, away, best = [], False, None
    for i, d in enumerate(d0):
        if d > START_FAR_M:
            away = True
        if away and d < START_NEAR_M:
            best = i if best is None or d < d0[best] else best
        elif best is not None:
            ends.append(best)
            best, away = None, d > START_FAR_M
    if best is not None:
        ends.append(best)
    return ends


def runs(mask):
    """(first, last) index pairs of the True runs."""
    out, start = [], None
    for i, f in enumerate(np.append(mask, False)):
        if f and start is None:
            start = i
        elif not f and start is not None:
            out.append((start, i - 1))
            start = None
    return out


def passes_at(tree, r, i, radius):
    """Separate passes of drive r within radius of sample i: (same, opposite) counts."""
    nb = np.sort(np.array(tree.query_ball_point([r['x'][i], r['y'][i]], radius)))
    same = opp = 0
    for grp in np.split(nb, np.where(np.diff(r['s'][nb]) > NEW_PASS_GAP_M)[0] + 1):
        j = grp[np.argmin(np.hypot(r['x'][grp] - r['x'][i], r['y'][grp] - r['y'][i]))]
        dh = heading_diff_deg(r['yaw'][j], r['yaw'][i])
        same += dh < SAME_DEG
        opp += dh > OPPOSITE_DEG
    return same, opp


def straight_legs(r):
    """Straight legs of drive r: (direction name, sample indices), by compass heading, merged across short gaps."""
    names = ('eastbound', 'northbound', 'westbound', 'southbound')
    deg = np.degrees(r['yaw'])
    k = np.round(deg / 90.0) % 4
    ok = heading_diff_deg(np.radians(deg), np.radians(k * 90.0)) < LEG_TOL_DEG
    legs = []
    for d in range(4):
        for a, b in runs(ok & (k == d)):
            legs.append([a, b, d])
    legs.sort()
    merged = []
    for a, b, d in legs:
        if merged and merged[-1][2] == d and r['s'][a] - r['s'][merged[-1][1]] < LEG_MIN_M:
            merged[-1][1] = b
        else:
            merged.append([a, b, d])
    out = []
    for a, b, d in merged:
        if r['s'][b] - r['s'][a] >= LEG_MIN_M:
            idx = np.arange(a, b + 1)
            out.append((names[d], idx[ok[idx] & (k[idx] == d)]))
    return out


def leg_place(r, idx, name):
    """Where a leg runs: its x range for north/south legs, its y range for east/west ones."""
    axis = 'x' if name in ('northbound', 'southbound') else 'y'
    lo, hi = np.percentile(r[axis][idx], [2, 98])
    return f'{axis} ≈ {lo:.0f}' if abs(hi - lo) < 1.5 else f'{axis} ≈ {lo:.0f} to {hi:.0f}'


def road_points(tree, pts, x, y, z):
    """Map points within ROAD_RADIUS_M horizontally and ROAD_DZ_M vertically of each (x, y, z)."""
    out = np.empty(len(x), dtype=int)
    for i, nb in enumerate(tree.query_ball_point(np.c_[x, y], ROAD_RADIUS_M)):
        out[i] = np.count_nonzero(np.abs(pts[nb, 2] - z[i]) < ROAD_DZ_M) if nb else 0
    return out


def reserved_section(sim_yaml):
    """(route name, first, last waypoint, length m, start xy, end xy) of the first route with reserved_reverse."""
    sim = yaml.safe_load(open(sim_yaml))
    for name, route in sim['routes'].items():
        if 'reserved_reverse' in route:
            a, b = route['reserved_reverse']
            wp = np.array(yaml.safe_load(open(ROOT / 'sim' / 'scenarios' / 'routes' / f'{name}.yaml'))['waypoints'])
            s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(wp[:, 0]), np.diff(wp[:, 1])))])
            return name, a, b, s[b] - s[a], wp[a, :2], wp[b, :2]
    raise SystemExit(f'{sim_yaml}: no route has reserved_reverse')


def map_pcd(map_yaml):
    """The pcd the map.yaml contract points at."""
    m = yaml.safe_load(open(map_yaml)).get('map') or {}
    if 'pcd' not in m:
        raise SystemExit(f'{map_yaml}: no map.pcd (not a map.yaml of the mapping backend)')
    return str(Path(map_yaml).parent / m['pcd'])


def plot(gm, gr, rv, legs, laps, out_png):
    plt.rcParams.update({'font.size': 10, 'text.color': INK, 'axes.labelcolor': INK2,
                         'xtick.color': INK2, 'ytick.color': INK2})
    fig, (a0, a1) = plt.subplots(1, 2, figsize=(13, 6.5), facecolor=SURFACE,
                                 gridspec_kw={'width_ratios': [1, 1.15]})
    m = 20.0
    box = (gr['x'].min() - m, gr['x'].max() + m, gr['y'].min() - m, gr['y'].max() + m)
    for ax in (a0, a1):
        ax.set_facecolor(SURFACE)
        ax.set_aspect('equal')
        ax.grid(color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        for sp in ax.spines.values():
            sp.set_color('#d0cfca')
        ax.set_xlabel('x [m] (map frame)')
        ax.set_ylabel('y [m]')
        ax.plot(gm['x'], gm['y'], color=BLUE, lw=2, label='mapping drive, all laps (ground truth)')
        ax.plot(gr['x'], gr['y'], color=ORANGE, lw=2, label='reverse_reuse drive (ground truth)')
    a0.set_title('Whole mapping drive', loc='left', fontsize=11)
    a0.add_patch(plt.Rectangle((box[0], box[2]), box[1] - box[0], box[3] - box[2], fill=False, ec=INK2, ls='--', lw=1))
    a1.set_title('Reserved section: offset to nearest mapping point', loc='left', fontsize=11)
    a1.set_xlim(box[0], box[1])
    a1.set_ylim(box[2], box[3])

    def arrows(x, y, color, every=45.0):
        s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
        for sv in np.arange(every / 2, s[-1], every):
            i = np.searchsorted(s, sv)
            j = np.searchsorted(s, sv + 1.0)
            if j >= len(x):
                break
            a1.annotate('', xy=(x[j], y[j]), xytext=(x[i], y[i]),
                        arrowprops=dict(arrowstyle='-|>', color=color, lw=1.5, mutation_scale=14))

    inside = (gm['x'] > box[0]) & (gm['x'] < box[1]) & (gm['y'] > box[2]) & (gm['y'] < box[3])
    first_lap = gm['t'] < gm['t'][0] + (gm['t'][-1] - gm['t'][0]) / max(1, laps)
    arrows(gm['x'][inside & first_lap], gm['y'][inside & first_lap], BLUE)
    arrows(gr['x'], gr['y'], ORANGE)
    for _, idx in legs:
        q = idx[len(idx) // 2]
        a1.annotate(f'{np.median(rv["d"][idx]):.1f} m', xy=(rv['x'][q], rv['y'][q]), xytext=(6, 4),
                    textcoords='offset points', fontsize=9, color=INK)
    for k, marker, label in ((0, 'o', 'reverse start'), (-1, 's', 'reverse end')):
        a1.plot(gr['x'][k], gr['y'][k], marker, ms=8, mfc=SURFACE, mec=ORANGE, mew=2)
        a1.annotate(label, (gr['x'][k], gr['y'][k]), xytext=(5, 3), textcoords='offset points', fontsize=9)
    h, lab = a0.get_legend_handles_labels()
    fig.legend(h, lab, loc='lower center', ncol=2, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out_png, dpi=110, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mapping_bag', help='raw bag of the mapping drive (with /eval/ground_truth)')
    ap.add_argument('reverse_bag', help='bag of the reverse_reuse drive (with /eval/ground_truth)')
    ap.add_argument('--map-yaml', required=True, help='map.yaml of the map localization uses')
    ap.add_argument('--sim', default=str(ROOT / 'sim' / 'config' / 'sim.yaml'))
    ap.add_argument('--out', required=True, help='folder for reverse_section.md/.json and reverse_vs_mapping.png')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    res = {'mapping_bag': args.mapping_bag, 'reverse_bag': args.reverse_bag, 'map_yaml': args.map_yaml}
    md = ['# Reserved reverse section against the mapping drive', '',
          f'Ground truth of `{args.mapping_bag}` and `{args.reverse_bag}`; map `{args.map_yaml}`. '
          f'Written by `eval/check_reverse_section.py`.', '']

    # ---- mapping drive ------------------------------------------------------------------------------------------
    gm_all = read_ground_truth(args.mapping_bag)
    gm, vm, still_before = moving_part(gm_all)
    rm, length = resample(gm)
    duration = gm['t'][-1] - gm['t'][0]
    ends = lap_ends(rm)
    laps = len(ends)
    stopped = float(np.sum(np.diff(gm['t'])[vm < MOVING_MPS]))
    lap1 = np.arange(0, ends[0] if ends else len(rm['s']))
    tree_m = cKDTree(np.c_[rm['x'], rm['y']])
    lane = np.array([passes_at(tree_m, rm, i, SAME_LANE_M) for i in lap1])
    road = np.array([passes_at(tree_m, rm, i, SAME_ROAD_M) for i in lap1])
    times = {int(n): float(np.mean(lane[:, 0] == n) * 100) for n in np.unique(lane[:, 0])}
    stretches = []
    for kind, mask in (('same direction, another lane', road[:, 0] > laps), ('both directions', road[:, 1] > 0)):
        for a, b in runs(mask):
            if rm['s'][lap1[b]] - rm['s'][lap1[a]] >= STRETCH_MIN_M:
                ia, ib = lap1[a], lap1[b]
                stretches.append({'kind': kind, 'lap_s_m': [float(rm['s'][ia]), float(rm['s'][ib])],
                                  'from': [float(rm['x'][ia]), float(rm['y'][ia])],
                                  'to': [float(rm['x'][ib]), float(rm['y'][ib])],
                                  'passes_same': int(road[a:b + 1, 0].max()), 'passes_opposite': int(road[a:b + 1, 1].max())})
    res['mapping'] = {'path_m': float(length), 'drive_s': float(duration), 'still_before_s': float(still_before),
                      'stopped_s': stopped, 'speed_median_mps': float(np.median(vm[vm > MOVING_MPS])),
                      'speed_max_mps': float(vm.max()), 'laps': laps,
                      'lap_m': [float(rm['s'][e] - (rm['s'][ends[i - 1]] if i else 0.0)) for i, e in enumerate(ends)],
                      'first_lap_driven_n_times_pct': times, 'extra_pass_stretches': stretches}
    md += ['## Mapping drive', '', '| | value |', '|---|---|',
           f'| path, time | {length:.1f} m in {duration:.1f} s (after {still_before:.1f} s standing at the start) |',
           f'| speed | median {res["mapping"]["speed_median_mps"]:.2f} m/s, max {vm.max():.2f} m/s; '
           f'stopped {stopped:.1f} s while driving |',
           f'| laps | {laps}: ' + ', '.join(f'{v:.1f} m' for v in res['mapping']['lap_m']) + ' |', '',
           f'Positions of the first lap by the number of same-direction passes within {SAME_LANE_M:.0f} m over the '
           'whole drive:', '', '| passes | share of the first lap |', '|---|---|']
    md += [f'| {n} | {p:.1f} % |' for n, p in times.items()]
    md += ['', f'Stretches of the first lap with extra passes on another lane of the same road (within '
           f'{SAME_ROAD_M:.0f} m), at least {STRETCH_MIN_M:.0f} m long:', '',
           '| lap metres | from | to | extra passes |', '|---|---|---|---|']
    md += [f'| {st["lap_s_m"][0]:.0f} to {st["lap_s_m"][1]:.0f} | ({st["from"][0]:.0f}, {st["from"][1]:.0f}) | '
           f'({st["to"][0]:.0f}, {st["to"][1]:.0f}) | {st["kind"]}: {st["passes_same"]} same, '
           f'{st["passes_opposite"]} opposite |' for st in stretches]

    # ---- reserved section -----------------------------------------------------------------------------------------
    route, a, b, planned, p_a, p_b = reserved_section(args.sim)
    res['reserved'] = {'route': route, 'waypoints': [a, b], 'planned_m': float(planned),
                       'from': p_a.tolist(), 'to': p_b.tolist()}
    md += ['', '## Reserved section', '',
           f'`{route}` waypoints {a} to {b}: {planned:.1f} m planned, ({p_a[0]:.1f}, {p_a[1]:.1f}) to '
           f'({p_b[0]:.1f}, {p_b[1]:.1f}).']

    # ---- reverse drive against the mapping drive --------------------------------------------------------------------
    gr, vr, _ = moving_part(read_ground_truth(args.reverse_bag))
    rv, length_r = resample(gr)
    duration_r = gr['t'][-1] - gr['t'][0]
    tree_all = cKDTree(np.c_[gm['x'], gm['y']])          # every 100 Hz mapping sample, not only the resampled ones
    rv['d'], k = tree_all.query(np.c_[rv['x'], rv['y']])
    rv['dh'] = heading_diff_deg(gm['yaw'][k], rv['yaw'])
    same = np.array([bool(nb) and bool((heading_diff_deg(gm['yaw'][nb], rv['yaw'][i]) < SAME_DEG).any())
                     for i, nb in enumerate(tree_all.query_ball_point(np.c_[rv['x'], rv['y']], SAME_LANE_M))])
    legs = straight_legs(rv)
    rows, in_leg = [], np.zeros(len(rv['s']), bool)
    for name, idx in legs:
        in_leg[idx] = True
        rows.append({'leg': f'{name}, {leg_place(rv, idx, name)}', 'length_m': len(idx) * SAMPLE_M,
                     'nearest_m': [float(rv['d'][idx].min()), float(np.median(rv['d'][idx])), float(rv['d'][idx].max())],
                     'heading_diff_median_deg': float(np.median(rv['dh'][idx]))})
    corners = np.where(~in_leg)[0]

    # ---- map coverage of the reverse lanes --------------------------------------------------------------------------
    pts = map_quality.read_pcd_xyz(map_pcd(args.map_yaml))
    tree_map = cKDTree(pts[:, :2])
    cov_r = road_points(tree_map, pts, rv['x'], rv['y'], rv['z'])
    ref = lap1[::4]
    cov_m = road_points(tree_map, pts, rm['x'][ref], rm['y'][ref], rm['z'][ref])
    coverage = float(np.mean(cov_r >= ROAD_MIN_POINTS) * 100)
    same_pct = float(np.mean(same) * 100)
    within = float(np.mean(rv['d'] <= SAME_LANE_M) * 100)
    ok = same_pct == 0.0 and coverage == 100.0
    res['reverse'] = {'from': [float(rv['x'][0]), float(rv['y'][0])], 'to': [float(rv['x'][-1]), float(rv['y'][-1])],
                      'path_m': float(length_r), 'drive_s': float(duration_r), 'speed_max_mps': float(vr.max()),
                      'legs': rows, 'corners_m': len(corners) * SAMPLE_M,
                      'corners_nearest_min_m': float(rv['d'][corners].min()) if len(corners) else None,
                      'corners_heading_diff_min_deg': float(rv['dh'][corners].min()) if len(corners) else None,
                      'within_3m_pct': within, 'same_direction_pct': same_pct, 'coverage_pct': coverage,
                      'road_points_reverse': {'median': float(np.median(cov_r)), 'min': int(cov_r.min())},
                      'road_points_mapped_lanes': {'median': float(np.median(cov_m)), 'min': int(cov_m.min())},
                      'pass': ok}
    md += ['', '## Reverse drive', '',
           f'({rv["x"][0]:.1f}, {rv["y"][0]:.1f}) to ({rv["x"][-1]:.1f}, {rv["y"][-1]:.1f}): {length_r:.1f} m in '
           f'{duration_r:.1f} s, max {vr.max():.2f} m/s.', '',
           '| leg | length [m] | nearest mapping point min / median / max [m] | heading difference, median [deg] |',
           '|---|---|---|---|']
    md += [f'| {r_["leg"]} | {r_["length_m"]:.0f} | {r_["nearest_m"][0]:.1f} / {r_["nearest_m"][1]:.1f} / '
           f'{r_["nearest_m"][2]:.1f} | {r_["heading_diff_median_deg"]:.0f} |' for r_ in rows]
    if len(corners):
        md.append(f'| corners | {len(corners) * SAMPLE_M:.0f} | at least {rv["d"][corners].min():.1f} | '
                  f'at least {rv["dh"][corners].min():.0f} |')
    md += ['', '## Proof', '', '| check | value | required |', '|---|---|---|',
           f'| same direction: a mapping sample within {SAME_LANE_M:.0f} m heading within {SAME_DEG:.0f} deg | '
           f'{same_pct:.1f} % | 0 % |',
           f'| map coverage: at least {ROAD_MIN_POINTS} road points within {ROAD_RADIUS_M:.0f} m | {coverage:.1f} % '
           f'(median {np.median(cov_r):.0f}, min {cov_r.min()}; mapped lanes median {np.median(cov_m):.0f}) | 100 % |',
           f'| for reference: nearest mapping point within {SAME_LANE_M:.0f} m | {within:.1f} % | (not a criterion) |',
           '', f'**{"PASS" if ok else "FAIL"}**', '']
    (out / 'reverse_section.md').write_text('\n'.join(md))
    (out / 'reverse_section.json').write_text(json.dumps(res, indent=1))
    plot(gm, gr, rv, legs, laps, out / 'reverse_vs_mapping.png')
    print('\n'.join(md))
    print(f'wrote {out}/reverse_section.md, reverse_section.json, reverse_vs_mapping.png')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
