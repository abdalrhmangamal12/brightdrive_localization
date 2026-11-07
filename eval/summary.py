#!/usr/bin/env python3
"""One table over every run in a results folder (eval/results/<run>/metrics.json from eval/loc_metrics.py).

  python3 eval/summary.py eval/results > eval/results/summary.md
"""
import json
import sys
from pathlib import Path


def g(d, *keys):
    for k in keys:
        if not isinstance(d, dict) or k not in d or d[k] is None:
            return None
        d = d[k]
    return d


def f(v, fmt='%.3f'):
    return '-' if v is None else (fmt % v if isinstance(v, (int, float)) else str(v))


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else 'eval/results')
    rows = []
    for m in sorted(root.glob('*/metrics.json')):
        r = json.loads(m.read_text())
        if 'checks' not in r:
            continue  # not a localization run (e.g. eval/results/mapping)
        failed = [c['check'] for c in r.get('checks', []) if not c['pass']]
        rej = g(r, 'sweeps', 'rejects') or {}
        rows.append([
            m.parent.name, str(r.get('scenario')),
            f(g(r, 'errors', 'ape_trans_rmse_m')), f(g(r, 'errors', 'lateral_m', 'p95')),
            f(g(r, 'errors', 'longitudinal_m', 'p95')), f(g(r, 'errors', 'heading_deg', 'p95'), '%.2f'),
            f(g(r, 'errors', 'horizontal_m', 'max')), f(r.get('convergence_s'), '%.1f'),
            f(g(r, 'latency_ms', 'for_check'), '%.2f'), f(g(r, 'input_age_ms', 'p99'), '%.0f'),
            f(g(r, 'nis_pose', 'mean'), '%.2f'),
            f(sum(v for k, v in rej.items() if k != 'accepted'), '%d') + ' / ' + f(sum(rej.values()), '%d') if rej else '-',
            f(g(r, 'gnss_outage', 'peak_drift_m')), f(g(r, 'gnss_outage', 'recovery_to_nominal_s'), '%.1f'),
            'pass' if not failed else 'FAIL: ' + ', '.join(failed),
        ])
    head = ['run', 'scenario', 'APE [m]', 'lat p95 [m]', 'lon p95 [m]', 'head p95 [deg]', 'max [m]', 'conv [s]',
            'latency p99 [ms] (1)', 'age p99 [ms]', 'NIS mean', 'rejected / matches', 'outage drift [m]',
            'outage recovery [s]', 'result']
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    for r in rows:
        print('| ' + ' | '.join(r) + ' |')
    print()
    print('(1) end to end on a wall-clock recorder; the node-measured processing latency when the run was recorded on '
          'sim time (10 ms steps).')


if __name__ == '__main__':
    main()
