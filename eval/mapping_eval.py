#!/usr/bin/env python3
"""The mapping result in one place: trajectory accuracy (APE, RPE vs ground truth) and map accuracy (cloud-to-cloud
against the GT reference map), with plots.

Trajectories are TUM files in the IMU frame written by the mapping backend (trajectory_optimized_tum.txt: the pose
graph; trajectory_fastlio_in_map_tum.txt: raw FAST-LIO moved into map once); ground truth comes from the mapping bag
through gt_to_tum.py. APE = translation part, no alignment (the map frame is georeferenced, so an alignment would hide
the georeferencing error); RPE = translation part over 1 m and 100 m of travel. C2C: eval/map_quality.py's nearest-
neighbour distances, both directions, for the map before and after the dynamic-object filter.

Inside the loc container (ROS sourced):
  python3 eval/mapping_eval.py --bag /bags/fastlio_run_20261105_133549 --map-dir /maps \
      --reference /maps/eval/gt_reference_map.pcd --out eval/results/mapping
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from evo.core import metrics, sync  # noqa: E402
from evo.core.metrics import PoseRelation, Unit  # noqa: E402
from evo.tools import file_interface  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_quality  # noqa: E402


def stats(a):
    a = np.asarray(a)
    return {'rmse': float(np.sqrt(np.mean(a ** 2))), 'mean': float(a.mean()), 'median': float(np.median(a)),
            'p95': float(np.percentile(a, 95)), 'max': float(a.max()), 'n': int(a.size)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--map-dir', required=True)
    ap.add_argument('--reference', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    md = Path(args.map_dir)
    gt_file = out / 'ground_truth_imu_tum.txt'
    if not gt_file.exists():
        subprocess.run([sys.executable, str(HERE / 'gt_to_tum.py'), args.bag, str(gt_file)], check=True)
    ref_all = file_interface.read_tum_trajectory_file(str(gt_file))

    res = {'bag': args.bag, 'map_dir': str(md), 'trajectories': {}, 'c2c': {}}
    curves = {}
    for name in ('trajectory_optimized_tum.txt', 'trajectory_fastlio_in_map_tum.txt'):
        f = md / name
        if not f.exists():
            continue
        est = file_interface.read_tum_trajectory_file(str(f))
        ref, est = sync.associate_trajectories(ref_all, est, max_diff=0.005)
        ape = metrics.APE(PoseRelation.translation_part)
        ape.process_data((ref, est))
        entry = {'poses': int(est.num_poses), 'ape_trans_m': stats(ape.error)}
        for delta in (1.0, 100.0):
            rpe = metrics.RPE(PoseRelation.translation_part, delta=delta, delta_unit=Unit.meters, all_pairs=False)
            try:
                rpe.process_data((ref, est))
                entry[f'rpe_trans_{delta:g}m'] = stats(rpe.error)
            except Exception as e:  # too short for 100 m pairs
                entry[f'rpe_trans_{delta:g}m'] = str(e)
        res['trajectories'][name] = entry
        curves[name] = (ref, est, ape.error)

    for name in ('town03.pcd', 'town03_static.pcd'):
        f = md / name
        if not f.exists():
            continue
        m = map_quality.read_pcd_xyz(str(f))
        r = map_quality.read_pcd_xyz(args.reference)
        res['c2c'][name] = {'map_to_reference': {k: float(v) for k, v in map_quality.nn_stats(m, r, 5.0).items()},
                            'reference_to_map': {k: float(v) for k, v in map_quality.nn_stats(r, m, 5.0).items()}}
    (out / 'metrics.json').write_text(json.dumps(res, indent=1))

    # Plots: APE over time, top view coloured by APE, C2C histogram.
    if curves:
        fig, ax = plt.subplots(figsize=(10, 3.5))
        for name, (ref, est, err) in curves.items():
            ax.plot(est.timestamps - est.timestamps[0], err, lw=0.7, label=name.replace('_tum.txt', ''))
        ax.set_xlabel('time [s]')
        ax.set_ylabel('APE trans [m]')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.savefig(out / 'ape_over_time.png', dpi=120, bbox_inches='tight')
        plt.close(fig)
        name = 'trajectory_optimized_tum.txt' if 'trajectory_optimized_tum.txt' in curves else next(iter(curves))
        ref, est, err = curves[name]
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(ref.positions_xyz[:, 0], ref.positions_xyz[:, 1], 'k-', lw=0.5, label='ground truth')
        sc = ax.scatter(est.positions_xyz[:, 0], est.positions_xyz[:, 1], c=err, s=3, cmap='viridis')
        fig.colorbar(sc, ax=ax, label='APE trans [m]')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title(name)
        fig.savefig(out / 'ape_xy.png', dpi=120, bbox_inches='tight')
        plt.close(fig)
    if (md / 'town03_static.pcd').exists():
        m = map_quality.read_pcd_xyz(str(md / 'town03_static.pcd'))
        r = map_quality.read_pcd_xyz(args.reference)
        from scipy.spatial import cKDTree
        d, _ = cKDTree(r).query(m, k=1, distance_upper_bound=5.0, workers=-1)
        d = d[np.isfinite(d)]
        fig, ax = plt.subplots(figsize=(7, 3.5))
        ax.hist(d, bins=200, range=(0, 1.0))
        ax.set_xlabel('distance to the GT reference map [m]')
        ax.set_ylabel('points')
        ax.set_title('town03_static.pcd -> reference (cloud to cloud)')
        fig.savefig(out / 'c2c_hist.png', dpi=120, bbox_inches='tight')
        plt.close(fig)

    lines = ['# Mapping evaluation', '', '| trajectory | poses | APE RMSE [m] | APE max [m] | RPE 1 m RMSE [m] | '
             'RPE 100 m RMSE [m] |', '|---|---|---|---|---|---|']
    for name, e in res['trajectories'].items():
        r100 = e['rpe_trans_100m']['rmse'] if isinstance(e['rpe_trans_100m'], dict) else float('nan')
        lines.append(f"| {name} | {e['poses']} | {e['ape_trans_m']['rmse']:.3f} | {e['ape_trans_m']['max']:.3f} | "
                     f"{e['rpe_trans_1m']['rmse']:.3f} | {r100:.3f} |")
    lines += ['', '| map | direction | mean [m] | median [m] | p95 [m] | > 0.5 m [%] |', '|---|---|---|---|---|---|']
    for name, c in res['c2c'].items():
        for d, s in c.items():
            lines.append(f"| {name} | {d} | {s['mean']:.3f} | {s['median']:.3f} | {s['p95']:.3f} | "
                         f"{s['over_0.5m']:.2f} |")
    (out / 'summary.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
