#!/usr/bin/env python3
"""Run one scenario file with ScenarioRunner, at real-time speed.

ScenarioRunner (--sync) ticks the world as fast as the server allows. The task measures a 1x real-time run, so
this script slows each tick down to the wall clock before handing over to ScenarioRunner. The tick length comes
from config/sim.yaml (tick_s), or --tick for one run. Runs inside the sim container (sim/scripts/run.sh
starts it through run_inside.sh):

  python3 -u sim/scenarios/run_scenario.py nominal                 # real time
  python3 -u sim/scenarios/run_scenario.py nominal --speed 0       # as fast as possible
  python3 -u sim/scenarios/run_scenario.py nominal --param Speed:12 --param TimeLimit:60
  python3 -u sim/scenarios/run_scenario.py nominal --tick 0.05     # one revolution per tick (assemble_sweeps 1)
"""
# Options, a summary at exit, loading ScenarioRunner as a module, Ctrl-C, file paths, wall-clock time.
import argparse
import atexit
import importlib.util
import signal
import sys
import time
from pathlib import Path

# CARLA API: we wrap its World.tick.
import carla

# sim/ folder; ScenarioRunner lives in the sim image; the scenario settings live in config/.
SIM = Path(__file__).resolve().parents[1]
SCENARIO_RUNNER = '/opt/scenario_runner/scenario_runner.py'
sys.path.insert(0, str(SIM / 'config'))
import settings  # noqa: E402  (sim/config/settings.py)


def pace_ticks(speed, tick):
    """Make every world.tick() also wait, so simulation time runs `speed` times as fast as real time
    (speed 0: no waiting). At exit, print how many ticks ran and the real-time factor they achieved."""
    original_tick = carla.World.tick
    clock = {'start': None, 'ticks': 0, 'total': 0, 'first': None, 'last': None, 'behind': 0}

    def paced_tick(world, *args, **kwargs):
        # The real tick first.
        frame = original_tick(world, *args, **kwargs)
        now = time.monotonic()
        clock['total'] += 1
        clock['first'] = clock['first'] or now
        clock['last'] = now
        if speed <= 0:
            return frame
        clock['ticks'] += 1
        if clock['start'] is None:
            clock['start'] = now
        # Where the wall clock should be after this many ticks.
        wait = clock['start'] + clock['ticks'] * tick / speed - now
        if wait > 0:
            time.sleep(wait)
        elif wait < -1.0:
            # More than 1 s behind (e.g. loading the town): restart the count instead of rushing to catch up.
            clock['start'], clock['ticks'] = time.monotonic(), 0
            clock['behind'] += 1
        return frame

    def summary():
        n, wall = clock['total'], (clock['last'] or 0) - (clock['first'] or 0)
        if n > 1 and wall > 0:
            print(f'ticks: {n} x {tick} s = {n * tick:.1f} s of simulation in {wall:.1f} s wall-clock -> real-time '
                  f'factor {n * tick / wall:.2f}x (target {"max" if speed <= 0 else f"{speed}x"}); '
                  f'fell >1 s behind {clock["behind"]} times', flush=True)

    atexit.register(summary)
    # Every caller of world.tick() in this process, including ScenarioRunner, now gets the paced version.
    carla.World.tick = paced_tick


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario', help='name in config/sim.yaml (file scenarios/xosc/<name>.xosc)')
    ap.add_argument('--speed', type=float, default=1.0, help='1 = real time, 2 = twice as fast, 0 = no waiting')
    ap.add_argument('--tick', type=float, help='world tick in seconds (default: tick_s in config/sim.yaml)')
    ap.add_argument('--param', action='append', default=[], help='override a .xosc parameter, e.g. Speed:12')
    a = ap.parse_args()
    # run_inside.sh starts us in the background, where bash switches Ctrl-C off: switch it back on until
    # ScenarioRunner installs its own handler (it then stops the scenario and removes its cars).
    signal.signal(signal.SIGINT, signal.default_int_handler)
    xosc = SIM / 'scenarios' / 'xosc' / f'{a.scenario}.xosc'
    if not xosc.exists():
        raise SystemExit(f'{xosc} not found: is "{a.scenario}" in config/sim.yaml? '
                         'Then run sim/scenarios/make_scenarios.py')
    tick = a.tick if a.tick else float(settings.config()['tick_s'])
    pace_ticks(a.speed, tick)
    # ScenarioRunner as a module, so its tick rate (a class attribute, 20 Hz) can be set before it runs.
    spec = importlib.util.spec_from_file_location('scenario_runner', SCENARIO_RUNNER)
    sr = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sr)
    sr.ScenarioRunner.frame_rate = 1.0 / tick
    # ScenarioRunner's command line, as if typed: synchronous mode, results on screen.
    sys.argv = [SCENARIO_RUNNER, '--openscenario', str(xosc), '--sync', '--output', '--timeout', '60']
    if a.param:
        sys.argv += ['--openscenarioparams', ','.join(a.param)]
    print(f'running {xosc.name} at {"max" if a.speed <= 0 else a.speed}x real time, tick {tick} s '
          f'({1.0 / tick:.0f} Hz)', flush=True)
    sys.exit(sr.main())


if __name__ == '__main__':
    main()
