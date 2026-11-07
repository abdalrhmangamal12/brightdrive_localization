#!/usr/bin/env python3
"""More traffic around a running scenario, for the "traffic x2" unseen variant: COUNT extra cars on the free spawn
points closest to our car, driven by CARLA's Traffic Manager (autopilot on the scenario's TM port). The scenario
keeps ticking the world (synchronous mode); this client only spawns, waits, and removes its cars on Ctrl-C.

Inside the sim container (carla module), while a scenario runs:
  python3 eval/spawn_traffic.py --count 40 [--tm-port 8000] [--max-dist 250]
"""
import argparse
import math
import signal
import sys
import time
from pathlib import Path

import carla
import yaml

SIM = Path(__file__).resolve().parents[1] / 'sim'
MODELS = ['vehicle.tesla.model3', 'vehicle.audi.a2', 'vehicle.toyota.prius', 'vehicle.nissan.micra',
          'vehicle.mini.cooper_s', 'vehicle.citroen.c3', 'vehicle.seat.leon', 'vehicle.dodge.charger_2020']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--count', type=int, default=40)
    ap.add_argument('--tm-port', type=int, default=8000)
    ap.add_argument('--max-dist', type=float, default=250.0, help='spawn points within this distance of our car [m]')
    ap.add_argument('--host', default='127.0.0.1')
    a = ap.parse_args()
    role = yaml.safe_load(open(SIM / 'config' / 'sensors.yaml'))['hero']['role_name']
    client = carla.Client(a.host, 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    hero = None
    for _ in range(300):
        hero = next((v for v in world.get_actors().filter('vehicle.*') if v.attributes.get('role_name') == role), None)
        if hero:
            break
        time.sleep(0.5)
    if hero is None:
        print('no hero vehicle found', flush=True)
        return 1
    here = hero.get_transform().location
    taken = [v.get_transform().location for v in world.get_actors().filter('vehicle.*')]
    spawns = sorted(world.get_map().get_spawn_points(), key=lambda s: s.location.distance(here))
    free = [s for s in spawns if s.location.distance(here) <= a.max_dist
            and all(s.location.distance(t) > 8.0 for t in taken)]
    lib = world.get_blueprint_library()
    tm = client.get_trafficmanager(a.tm_port)
    batch = []
    for i, sp in enumerate(free[:a.count]):
        bp = lib.find(MODELS[i % len(MODELS)])
        bp.set_attribute('role_name', 'extra_traffic')
        batch.append(carla.command.SpawnActor(bp, sp).then(
            carla.command.SetAutopilot(carla.command.FutureActor, True, a.tm_port)))
    ids = [r.actor_id for r in client.apply_batch_sync(batch, False) if not r.error]
    print(f'spawned {len(ids)} of {a.count} extra cars within {a.max_dist:.0f} m of the hero '
          f'({len(free)} free spawn points), autopilot on TM port {a.tm_port}', flush=True)

    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while not stop:
        if not any(v.id == hero.id for v in world.get_actors().filter('vehicle.*')):
            break  # the scenario ended
        time.sleep(1.0)
    client.apply_batch([carla.command.DestroyActor(i) for i in ids])
    print(f'removed {len(ids)} extra cars', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
