#!/usr/bin/env python3
"""Check the sim container against the CARLA server. Run inside the sim container:

  python3 /ws/src/brightdrive_localization/sim/scripts/check_client.py

connect with a timeout -> client and server versions must match -> Town03: load it unless the server already has
it (it starts there, docker/UserEngine.ini) -> synchronous mode
with fixed_delta_seconds 0.01, 100 ticks (wall time, real-time factor) -> asynchronous mode restored in a
finally block, whatever happened -> PASS or FAIL on the last line. Exit code 0 = PASS, 1 = FAIL.
"""
import argparse
import sys
import time

import carla


def check(a):
    """True when every check passed."""
    client = carla.Client(a.host, a.port)
    client.set_timeout(a.timeout)
    client_version = client.get_client_version()
    try:
        server_version = client.get_server_version()
    except RuntimeError as err:
        print(f"no answer from CARLA at {a.host}:{a.port} within {a.timeout} s ({err})")
        return False
    print(f"client {client_version} | server {server_version}")
    if client_version != server_version:
        print("client and server versions differ")
        return False
    print("versions match")

    client.set_timeout(max(a.timeout, 180.0))  # loading a town takes tens of seconds
    world = client.get_world()
    loaded = world.get_map().name              # e.g. 'Carla/Maps/Town03'
    if loaded.rsplit('/', 1)[-1] == a.town:
        print(f"{loaded} already loaded, no reload")
    else:
        print(f"server has {loaded}, loading {a.town}")
        t0 = time.monotonic()
        world = client.load_world(a.town)
        print(f"{world.get_map().name} loaded in {time.monotonic() - t0:.1f} s")
    client.set_timeout(a.timeout)

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True        # the server advances only when this client calls world.tick()
    settings.fixed_delta_seconds = a.dt     # each tick advances sim time by exactly dt
    world.apply_settings(settings)
    try:
        sim0 = world.get_snapshot().timestamp.elapsed_seconds
        t0 = time.monotonic()               # wall clock; sim time stands still between ticks
        for _ in range(a.ticks):
            world.tick()
        wall = time.monotonic() - t0
        sim = world.get_snapshot().timestamp.elapsed_seconds - sim0
        print(f"{a.ticks} ticks x {a.dt} s: sim {sim:.3f} s in wall {wall:.3f} s -> {sim / wall:.2f}x real time "
              f"({1e3 * wall / a.ticks:.1f} ms per tick, no sensors attached)")
        ok = abs(sim - a.ticks * a.dt) < 1e-3
        if not ok:
            print(f"sim time advanced {sim:.3f} s, expected {a.ticks * a.dt:.3f} s")
        return ok
    finally:
        # A server left in synchronous mode with nobody ticking freezes for every other client.
        restore = original
        restore.synchronous_mode = False
        world.apply_settings(restore)
        now = world.get_settings()
        print(f"restored: synchronous_mode={now.synchronous_mode}, fixed_delta_seconds={now.fixed_delta_seconds}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="CARLA host (host networking: 127.0.0.1)")
    ap.add_argument("--port", type=int, default=2000, help="CARLA RPC port")
    ap.add_argument("--timeout", type=float, default=60.0, help="RPC timeout in seconds (load_world gets 180 s)")
    ap.add_argument("--town", default="Town03")
    ap.add_argument("--dt", type=float, default=0.01, help="fixed_delta_seconds for the tick test")
    ap.add_argument("--ticks", type=int, default=100)
    ok = check(ap.parse_args())
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
