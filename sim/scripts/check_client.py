#!/usr/bin/env python3
"""Check the ros container against the CARLA server. Run inside the ros container:

  python3 /ws/src/brightdrive_localization/sim/scripts/check_client.py

connect with a timeout -> client and server versions must match -> load Town03 (load time) -> synchronous mode
with fixed_delta_seconds 0.01, 100 ticks (wall time, real-time factor) -> asynchronous mode restored in a
finally block, whatever happened. Exit code 0 = every PASS, 1 = a FAIL.
"""
import argparse
import sys
import time

import carla


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="CARLA host (host networking: 127.0.0.1)")
    ap.add_argument("--port", type=int, default=2000, help="CARLA RPC port")
    ap.add_argument("--timeout", type=float, default=60.0, help="RPC timeout in seconds (load_world gets 180 s)")
    ap.add_argument("--town", default="Town03")
    ap.add_argument("--dt", type=float, default=0.01, help="fixed_delta_seconds for the tick test")
    ap.add_argument("--ticks", type=int, default=100)
    a = ap.parse_args()

    client = carla.Client(a.host, a.port)
    client.set_timeout(a.timeout)
    client_version = client.get_client_version()
    try:
        server_version = client.get_server_version()
    except RuntimeError as err:
        print(f"FAIL: no answer from CARLA at {a.host}:{a.port} within {a.timeout} s ({err})")
        return 1
    print(f"client {client_version} | server {server_version}")
    if client_version != server_version:
        print("FAIL: client and server versions differ")
        return 1
    print("PASS: versions match")

    client.set_timeout(max(a.timeout, 180.0))  # loading a town takes tens of seconds
    t0 = time.monotonic()
    world = client.load_world(a.town)
    load_s = time.monotonic() - t0
    client.set_timeout(a.timeout)
    print(f"{world.get_map().name} loaded in {load_s:.1f} s")

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
        print(("PASS" if ok else "FAIL") + f": sim time advanced {sim:.3f} s, expected {a.ticks * a.dt:.3f} s")
        return 0 if ok else 1
    finally:
        # A server left in synchronous mode with nobody ticking freezes for every other client.
        restore = original
        restore.synchronous_mode = False
        world.apply_settings(restore)
        print(f"restored: synchronous_mode={world.get_settings().synchronous_mode}, "
              f"fixed_delta_seconds={world.get_settings().fixed_delta_seconds}")


if __name__ == "__main__":
    sys.exit(main())
