#!/usr/bin/env python3
"""Metrics of one localization run against ground truth, from one recorded bag.

The bag holds the localizer's outputs (/localization/pose, /status, /debug/match), the IMU, and the scenario's
/eval/* topics (localization_replay.launch.py records exactly these). Ground truth is used here only.

Per run:
  - position error in the ground-truth heading frame: longitudinal (along), lateral (left), vertical; horizontal
    norm; RMSE, p95, max; heading error (yaw difference, wrapped); APE trans_part as evo computes it (3-D RMSE).
    Statistics cover the converged part of the run (from the convergence time on).
  - convergence: first output, and the first time from which the horizontal error stays < 0.5 m and the heading
    error < 2 deg for 3 s; measured from the first IMU sample of the run (relocalization: from /initialpose).
  - latency: end to end = recorder receive time of /localization/pose minus that of the /imu/data sample with the
    same stamp (one recorder clock); processing = output_latency_ms from the status (IMU callback to publish).
  - input age: match_age_ms per sweep (filter time minus sweep end when the match is applied); its trend in
    ms per minute and early/late medians (flat = nothing accumulates); match time per sweep.
  - pose NIS (mean, share above the 6-DoF 95 % / 99.9 % bounds), reject counts per code, GNSS counters.
  - health timeline from /localization/status.
  - gnss_outage: window from /eval/gnss_outage (WARN = outage), peak horizontal drift in it, sigma over time,
    DEGRADED share in the window, NOMINAL recovery time after it, FAILED count.
  - relocalization: the /initialpose offset (/eval/init_offset), convergence time, accuracy after convergence.
  - high_speed_turns: speed, yaw rate, lateral acceleration from ground truth; heading error and latency in the
    fast parts.
Pass/fail against the thresholds below. Writes metrics.json, summary.md and PNG plots into --out.

Inside the loc container (ROS sourced):
  python3 eval/loc_metrics.py <run_bag> --scenario <name> --out eval/results/<name>
"""
import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

# Pass/fail thresholds (day-0 plan). The first four are the freeway, passenger-vehicle localization requirements
# of Reid et al., "Localization Requirements for Autonomous Vehicles", SAE Int. J. CAV 2019 (95 % lateral and
# longitudinal accuracy, lateral alert limit); the rest follow from them and from the output rate.
THRESHOLDS = {
    'lateral_p95_m': (0.20, 'lateral 95 % accuracy for a passenger car on a freeway lane (Reid et al. 2019)'),
    'longitudinal_p95_m': (0.48, 'longitudinal 95 % accuracy, same source; along-track error is less critical'),
    'heading_p95_deg': (0.5, '0.5 deg at 20 m look-ahead is 0.17 m lateral, inside the lateral budget'),
    'horizontal_max_m': (0.57, 'lateral alert limit (Reid et al. 2019): never leave the lane margin'),
    'outage_peak_drift_m': (0.57, 'the alert limit also holds while GNSS is gone'),
    'outage_recovery_s': (5.0, 'NOMINAL within 5 s after GNSS returns: a few fixes to re-confirm'),
    'init_convergence_s': (10.0, 'a bad initial pose is corrected within 10 s of data'),
    'high_speed_heading_p95_deg': (1.0, 'relaxed heading bound for the 15 m/s turns'),
    'latency_p99_ms': (20.0, 'one output period at 50 Hz: no consumer sees a pose older than one step'),
}
CHI2_6_95, CHI2_6_999 = 12.59, 22.46
REJECT_NAMES = ['accepted', 'too_few_points', 'not_converged', 'error_per_inlier', 'inlier_ratio', 'no_history',
                'jump', 'age', 'gated']
CONV_POS_M, CONV_HEAD_DEG, CONV_HOLD_S = 0.5, 2.0, 3.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def level(status):
    """DiagnosticStatus.level as an int (rclpy gives the byte field as bytes)."""
    v = status.level
    return v[0] if isinstance(v, (bytes, bytearray)) else int(v)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_bag(path, topics):
    with open(os.path.join(path, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    msgs = {t: [] for t in topics}
    classes = {t: get_message(types[t]) for t in wanted}
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        msgs[topic].append((t_recv * 1e-9, deserialize_message(data, classes[topic])))
    return msgs


def q_stats(v):
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return {'n': 0}
    a = np.abs(v)
    return {'n': int(v.size), 'rmse': float(np.sqrt(np.mean(v * v))), 'mean_abs': float(a.mean()),
            'p50': float(np.percentile(a, 50)), 'p95': float(np.percentile(a, 95)),
            'p99': float(np.percentile(a, 99)), 'max': float(a.max())}


def interp_gt(gt_t, gt_p, gt_yaw, t):
    i = np.searchsorted(gt_t, t)
    ok = (i > 0) & (i < len(gt_t))
    i = np.clip(i, 1, len(gt_t) - 1)
    t0, t1 = gt_t[i - 1], gt_t[i]
    a = np.where(t1 > t0, (t - t0) / np.maximum(t1 - t0, 1e-12), 0.0)
    ok &= (t1 - t0) < 0.05
    p = gt_p[i - 1] + a[:, None] * (gt_p[i] - gt_p[i - 1])
    dy = np.array([wrap(v) for v in gt_yaw[i] - gt_yaw[i - 1]])
    yaw = gt_yaw[i - 1] + a * dy
    return p, yaw, ok


def segments(times, labels):
    """[(t_start, t_end, label)] runs of equal labels."""
    out = []
    for t, l in zip(times, labels):
        if out and out[-1][2] == l:
            out[-1][1] = t
        else:
            out.append([t, t, l])
    return [tuple(s) for s in out]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--scenario', default='nominal')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    m = read_bag(args.bag, ['/eval/ground_truth', '/localization/pose', '/localization/status',
                            '/localization/debug/match', '/imu/data', '/eval/gnss_outage', '/eval/init_offset',
                            '/initialpose'])
    if not m['/eval/ground_truth'] or not m['/localization/pose']:
        raise SystemExit('the bag needs /eval/ground_truth and /localization/pose')

    gt = sorted(((stamp(g.header), g) for _, g in m['/eval/ground_truth']), key=lambda r: r[0])
    gt_t = np.array([r[0] for r in gt])
    gt_p = np.array([[g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.position.z] for _, g in gt])
    gt_yaw = np.array([yaw_of((g.pose.pose.orientation.x, g.pose.pose.orientation.y, g.pose.pose.orientation.z,
                               g.pose.pose.orientation.w)) for _, g in gt])
    gt_v = np.array([[g.twist.twist.linear.x, g.twist.twist.linear.y, g.twist.twist.linear.z] for _, g in gt])
    gt_w = np.array([g.twist.twist.angular.z for _, g in gt])

    est = [(stamp(p.header), t_recv, p) for t_recv, p in m['/localization/pose']]
    est.sort(key=lambda r: r[0])
    e_t = np.array([r[0] for r in est])
    e_p = np.array([[r[2].pose.pose.position.x, r[2].pose.pose.position.y, r[2].pose.pose.position.z] for r in est])
    e_yaw = np.array([yaw_of((r[2].pose.pose.orientation.x, r[2].pose.pose.orientation.y,
                              r[2].pose.pose.orientation.z, r[2].pose.pose.orientation.w)) for r in est])
    e_sig = np.array([math.sqrt(max(0.0, (r[2].pose.covariance[0] + r[2].pose.covariance[7]
                                          + r[2].pose.covariance[14]) / 3.0)) for r in est])
    e_sig_yaw = np.array([math.degrees(math.sqrt(max(0.0, r[2].pose.covariance[35]))) for r in est])

    g_p, g_yaw, ok = interp_gt(gt_t, gt_p, gt_yaw, e_t)
    e_t, e_p, e_yaw, e_sig, e_sig_yaw, g_p, g_yaw = (x[ok] for x in (e_t, e_p, e_yaw, e_sig, e_sig_yaw, g_p, g_yaw))
    d = e_p - g_p
    c, s = np.cos(g_yaw), np.sin(g_yaw)
    lon = d[:, 0] * c + d[:, 1] * s
    lat = -d[:, 0] * s + d[:, 1] * c
    horiz = np.hypot(d[:, 0], d[:, 1])
    head = np.degrees([wrap(v) for v in e_yaw - g_yaw])

    # Reference time for convergence: the first IMU sample (relocalization: the /initialpose stamp).
    imu_stamps = [stamp(i.header) for _, i in m['/imu/data']]
    t_ref0 = min(imu_stamps) if imu_stamps else gt_t[0]
    if args.scenario == 'relocalization' and m['/initialpose']:
        t_ref0 = stamp(m['/initialpose'][0][1].header)
    good = (horiz < CONV_POS_M) & (np.abs(head) < CONV_HEAD_DEG)
    t_conv = None
    j = 0
    while j < len(e_t):
        if not good[j]:
            j += 1
            continue
        k = j
        while k < len(e_t) and good[k]:
            k += 1
        if e_t[k - 1] - e_t[j] >= CONV_HOLD_S or k == len(e_t):
            t_conv = e_t[j]
            break
        j = k
    conv = e_t >= (t_conv if t_conv is not None else e_t[-1] + 1)

    res = {'bag': args.bag, 'scenario': args.scenario, 'outputs': int(len(e_t)),
           'duration_s': float(e_t[-1] - e_t[0]) if len(e_t) else 0.0,
           'first_output_s': float(e_t[0] - t_ref0) if len(e_t) else None,
           'convergence_s': float(t_conv - t_ref0) if t_conv is not None else None}
    res['errors'] = {'longitudinal_m': q_stats(lon[conv]), 'lateral_m': q_stats(lat[conv]),
                     'vertical_m': q_stats(d[conv, 2]), 'horizontal_m': q_stats(horiz[conv]),
                     'heading_deg': q_stats(head[conv]),
                     'ape_trans_rmse_m': float(np.sqrt(np.mean(np.sum(d[conv] ** 2, axis=1)))) if conv.any()
                     else None}
    res['errors_whole_run'] = {'horizontal_m': q_stats(horiz), 'heading_deg': q_stats(head)}

    # Latency on the recorder's clock: pose receive minus IMU receive for the same stamp.
    imu_recv = {round(stamp(i.header), 6): t for t, i in m['/imu/data']}
    lat_ms = [1e3 * (r[1] - imu_recv[round(r[0], 6)]) for r in est if round(r[0], 6) in imu_recv]
    status = [(stamp(a.header), {kv.key: kv.value for kv in a.status[0].values}, level(a.status[0]))
              for _, a in m['/localization/status'] if a.status]
    proc = [float(v['output_latency_ms']) for _, v, _ in status if v.get('output_latency_ms') not in (None, '-')]
    # A recorder on sim time (run.sh records with --use-sim-time) stamps on the 10 ms /clock grid: its receive
    # times cannot resolve a sub-millisecond latency. Then the node's own measurement is the latency figure.
    lat_arr = np.asarray(lat_ms)
    sim_clock = lat_arr.size > 0 and np.mean(np.abs(np.round(lat_arr / 10.0) * 10.0 - lat_arr) < 1e-3) > 0.9
    res['latency_ms'] = {'end_to_end': q_stats(lat_ms), 'recorder_clock': 'sim (10 ms steps)' if sim_clock
                         else 'wall', 'processing_max_reported':
                         max((float(v.get('output_latency_max_ms', 0)) for _, v, _ in status), default=None),
                         'processing': q_stats(proc)}
    res['latency_ms']['for_check'] = (res['latency_ms']['processing'].get('p99') if sim_clock
                                      else res['latency_ms']['end_to_end'].get('p99'))

    # Per-sweep stream: [t_ref, time_ms, inliers, source, ratio, e/inl, nis, accepted, code, age_ms]
    mt = np.array([list(a.data) for _, a in m['/localization/debug/match']]) if m['/localization/debug/match'] \
        else np.zeros((0, 10))
    track = mt[mt[:, 7] != 2] if len(mt) else mt
    res['sweeps'] = {'matched': int(len(track))}
    if len(track):
        codes = track[:, 8].astype(int)
        res['sweeps']['rejects'] = {REJECT_NAMES[c]: int((codes == c).sum()) for c in range(len(REJECT_NAMES))}
        reached = (codes == 0) | (codes == 8)
        nis = track[reached, 6]
        res['nis_pose'] = {'n': int(nis.size), 'mean': float(nis.mean()) if nis.size else None,
                           'share_above_95': float((nis > CHI2_6_95).mean()) if nis.size else None,
                           'share_above_999': float((nis > CHI2_6_999).mean()) if nis.size else None}
        age = track[:, 9]
        tt = track[:, 0]
        slope = float(np.polyfit((tt - tt[0]) / 60.0, age, 1)[0]) if len(tt) > 10 else None
        third = max(1, len(age) // 3)
        res['input_age_ms'] = dict(q_stats(age), trend_ms_per_min=slope,
                                   first_third_p50=float(np.median(age[:third])),
                                   last_third_p50=float(np.median(age[-third:])))
        res['match_time_ms'] = q_stats(track[:, 1])
        res['error_per_inlier'] = q_stats(track[codes == 0, 5])
        res['inlier_ratio'] = q_stats(track[codes == 0, 4])
    if status:
        last = status[-1][1]
        res['status_last'] = {k: last[k] for k in last if k.startswith(('gnss.', 'sweeps_', 'init', 'time_to_tracking', 'failures', 'last_fail', 'match.'))}
        st_t = np.array([s[0] for s in status])
        labels = [s[1].get('lifecycle', '?') + (' ' + s[1].get('health', '') if s[1].get('lifecycle') ==
                                                 'TRACKING' else '') for s in status]
        res['health_timeline'] = [(round(a - t_ref0, 2), round(b - t_ref0, 2), l) for a, b, l in segments(st_t,
                                                                                                         labels)]
        res['failed_status_count'] = int(sum('FAILED' in l for l in labels))

    # Scenario sections.
    outage = None
    if m['/eval/gnss_outage']:
        warn = [stamp(a.header) for _, a in m['/eval/gnss_outage'] if a.status and level(a.status[0]) == 1]
        if warn:
            outage = (min(warn), max(warn))
    if outage and status:
        ts, te = outage
        win = (e_t >= ts) & (e_t <= te + 2.0)
        sw = [(t, s[1].get('health')) for t, s in zip(st_t, status) if ts + 1.5 <= t <= te]
        after = [t for t, s in zip(st_t, status) if t > te and s[1].get('health') == 'NOMINAL']
        res['gnss_outage'] = {'start_s': round(ts - t_ref0, 2), 'end_s': round(te - t_ref0, 2),
                              'peak_drift_m': float(horiz[win].max()) if win.any() else None,
                              'sigma_pos_max_m': float(e_sig[win].max()) if win.any() else None,
                              'degraded_share_in_window': float(np.mean([h == 'DEGRADED' for _, h in sw]))
                              if sw else None,
                              'recovery_to_nominal_s': float(after[0] - te) if after else None}
    if args.scenario == 'relocalization':
        off = m['/eval/init_offset'][0][1] if m['/eval/init_offset'] else None
        last = e_t >= e_t[-1] - 10.0
        res['relocalization'] = {'init_offset_m': float(math.hypot(off.vector.x, off.vector.y)) if off else None,
                                 'init_heading_offset_deg': float(off.vector.z) if off else None,
                                 'final_10s_horizontal_m': q_stats(horiz[last]),
                                 'final_10s_heading_deg': q_stats(head[last])}
    speed = np.linalg.norm(gt_v[:, :2], axis=1)
    sp_e = np.interp(e_t, gt_t, speed)
    yr_e = np.interp(e_t, gt_t, gt_w)
    res['dynamics'] = {'speed_max_mps': float(speed.max()), 'yaw_rate_max_dps': float(np.degrees(np.abs(gt_w).max())),
                       'lat_accel_max_mps2': float(np.abs(speed * gt_w).max())}
    if args.scenario == 'high_speed_turns':
        fast = conv & (sp_e > 12.0)
        res['high_speed'] = {'samples': int(fast.sum()), 'heading_deg': q_stats(head[fast]),
                             'lateral_m': q_stats(lat[fast])}

    # Pass / fail.
    checks = []

    def check(name, value, key, smaller=True):
        thr, why = THRESHOLDS[key]
        passed = value is not None and (value <= thr if smaller else value >= thr)
        checks.append({'check': name, 'value': value, 'threshold': thr, 'pass': bool(passed), 'why': why})
    e = res['errors']
    if args.scenario == 'relocalization':
        check('convergence time [s]', res['convergence_s'], 'init_convergence_s')
    check('lateral p95 [m]', e['lateral_m'].get('p95'), 'lateral_p95_m')
    check('longitudinal p95 [m]', e['longitudinal_m'].get('p95'), 'longitudinal_p95_m')
    if args.scenario == 'high_speed_turns':
        check('heading p95, fast parts [deg]', res['high_speed']['heading_deg'].get('p95'),
              'high_speed_heading_p95_deg')
    else:
        check('heading p95 [deg]', e['heading_deg'].get('p95'), 'heading_p95_deg')
    check('horizontal max [m]', e['horizontal_m'].get('max'), 'horizontal_max_m')
    check('latency p99 [ms]' + (' (node-measured, sim-time recorder)' if res['latency_ms']['recorder_clock']
                                 != 'wall' else ''), res['latency_ms']['for_check'], 'latency_p99_ms')
    if 'gnss_outage' in res:
        g = res['gnss_outage']
        check('outage peak drift [m]', g['peak_drift_m'], 'outage_peak_drift_m')
        check('NOMINAL after outage [s]', g['recovery_to_nominal_s'], 'outage_recovery_s')
        checks.append({'check': 'DEGRADED through the window', 'value': g['degraded_share_in_window'],
                       'threshold': 1.0, 'pass': g['degraded_share_in_window'] == 1.0,
                       'why': 'one absolute source lost = DEGRADED by definition'})
        checks.append({'check': 'never FAILED', 'value': res.get('failed_status_count'), 'threshold': 0,
                       'pass': res.get('failed_status_count') == 0, 'why': 'the outage is expected, not a failure'})
    res['checks'] = checks

    with open(out / 'metrics.json', 'w') as f:
        json.dump(res, f, indent=1)
    lines = [f'# {args.scenario}: {Path(args.bag).name}', '', '| check | value | threshold | pass |',
             '|---|---|---|---|']
    for ch in checks:
        v = ch['value']
        lines.append(f"| {ch['check']} | {v:.3f} | {ch['threshold']} | {'yes' if ch['pass'] else 'NO'} |"
                     if isinstance(v, float) else f"| {ch['check']} | {v} | {ch['threshold']} | "
                     f"{'yes' if ch['pass'] else 'NO'} |")
    (out / 'summary.md').write_text('\n'.join(lines) + '\n')

    # Plots.
    tr = e_t - t_ref0
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(gt_p[:, 0], gt_p[:, 1], 'k-', lw=1, label='ground truth')
    ax.plot(e_p[:, 0], e_p[:, 1], 'C1-', lw=1, label='localization')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend()
    ax.set_title(f'{args.scenario}: trajectory (map)')
    fig.savefig(out / 'trajectory.png', dpi=120, bbox_inches='tight')
    plt.close(fig)

    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axs[0].plot(tr, lat, lw=0.7, label='lateral')
    axs[0].plot(tr, lon, lw=0.7, label='longitudinal')
    axs[0].set_ylabel('error [m]')
    axs[0].legend(loc='upper right')
    axs[1].plot(tr, head, lw=0.7, color='C2')
    axs[1].set_ylabel('heading error [deg]')
    axs[2].plot(tr, horiz, lw=0.7, color='C3', label='|horizontal error|')
    axs[2].plot(tr, 3 * e_sig, lw=0.7, color='C0', label='3 sigma (pose covariance)')
    axs[2].set_ylabel('[m]')
    axs[2].legend(loc='upper right')
    if len(mt):
        axs[3].plot(mt[:, 0] - t_ref0, mt[:, 9], '.', ms=1.5, label='match age [ms]')
        axs[3].plot(mt[:, 0] - t_ref0, mt[:, 1], '.', ms=1.5, label='match time [ms]')
        axs[3].legend(loc='upper right')
    axs[3].set_xlabel('time [s]')
    for a in axs:
        a.grid(alpha=0.3)
        if outage:
            a.axvspan(outage[0] - t_ref0, outage[1] - t_ref0, color='orange', alpha=0.15)
        if t_conv is not None:
            a.axvline(t_conv - t_ref0, color='k', lw=0.5, ls='--')
    axs[0].set_title(f'{args.scenario}: errors vs ground truth (shaded: GNSS outage, dashed: converged)')
    fig.savefig(out / 'errors.png', dpi=120, bbox_inches='tight')
    plt.close(fig)

    if status:
        hmap = {'NOMINAL': 0, 'DEGRADED': 1, 'DEAD_RECKONING': 2, 'FAILED': 3}
        fig, ax = plt.subplots(2, 1, figsize=(10, 4), sharex=True)
        hv = [hmap.get(s[1].get('health'), np.nan) if s[1].get('lifecycle') == 'TRACKING' else -1 for s in status]
        ax[0].step(st_t - t_ref0, hv, where='post')
        ax[0].set_yticks([-1, 0, 1, 2, 3])
        ax[0].set_yticklabels(['not tracking', 'NOMINAL', 'DEGRADED', 'DEAD_RECK.', 'FAILED'])
        gage = [float(s[1]['gnss_age_s']) if s[1].get('gnss_age_s', '-') != '-' else np.nan for s in status]
        ax[1].plot(st_t - t_ref0, gage)
        ax[1].set_ylabel('gnss_age_s')
        ax[1].set_xlabel('time [s]')
        for a in ax:
            a.grid(alpha=0.3)
            if outage:
                a.axvspan(outage[0] - t_ref0, outage[1] - t_ref0, color='orange', alpha=0.15)
        ax[0].set_title(f'{args.scenario}: health timeline')
        fig.savefig(out / 'health.png', dpi=120, bbox_inches='tight')
        plt.close(fig)

    if len(track):
        reached = (track[:, 8] == 0) | (track[:, 8] == 8)
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(track[reached, 0] - t_ref0, track[reached, 6], '.', ms=2)
        ax.axhline(CHI2_6_95, color='C1', lw=0.7, label='chi2(6) 95 %')
        ax.axhline(CHI2_6_999, color='C3', lw=0.7, label='gate 99.9 %')
        ax.set_ylabel('pose NIS')
        ax.set_xlabel('time [s]')
        ax.set_yscale('log')
        ax.legend()
        ax.grid(alpha=0.3)
        fig.savefig(out / 'nis.png', dpi=120, bbox_inches='tight')
        plt.close(fig)

    if args.scenario == 'high_speed_turns':
        fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        ax[0].plot(tr, sp_e)
        ax[0].set_ylabel('speed [m/s]')
        ax[1].plot(tr, np.degrees(yr_e), label='yaw rate [deg/s]')
        ax[1].plot(tr, sp_e * yr_e, label='lateral acc. [m/s^2]')
        ax[1].legend()
        ax[2].plot(tr, head, lw=0.7, color='C2')
        ax[2].set_ylabel('heading error [deg]')
        ax[2].set_xlabel('time [s]')
        for a in ax:
            a.grid(alpha=0.3)
        fig.savefig(out / 'dynamics.png', dpi=120, bbox_inches='tight')
        plt.close(fig)

    print((out / 'summary.md').read_text())
    print(json.dumps({k: res[k] for k in ('convergence_s', 'first_output_s')}, indent=None))


if __name__ == '__main__':
    main()
