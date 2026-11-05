#!/usr/bin/env python3
"""The start pose the estimator is given in the re-localization scenario: the scenario's start pose plus a known
offset ("startup with an approximate position and heading, including a 5 m position offset and 30-degree heading
offset").

Nothing here comes from the live simulation. The start pose is read from the scenario itself: sim.yaml `start_at`
names a waypoint of the route file, and the heading is the direction of the route segment that starts there. The
.xosc teleports the car to that same waypoint, so this is the true start pose up to the car settling on the road.
The offsets are the scenario's init_offset_m, init_offset_dir_deg and init_heading_offset_deg; `random` draws the
direction and the sign of the heading offset from `seed`, so "random" is still the same every run.

  /initialpose       geometry_msgs/PoseWithCovarianceStamped, frame map, once, QoS transient_local depth 1,
                     stamped with the first /clock: position = waypoint + init_offset_m along init_offset_dir_deg
                     (relative to the heading: 0 = ahead, 90 = left); yaw = segment heading +
                     init_heading_offset_deg. Covariance diagonal: sigma_x = sigma_y = init_offset_m, sigma_z = 1 m,
                     sigma_roll = sigma_pitch = 5 deg, sigma_yaw = |init_heading_offset_deg| (the offsets
                     themselves, so the estimator's gate accepts the true pose at 1 sigma).
  /eval/init_offset  geometry_msgs/Vector3Stamped, once, transient_local: x, y = the applied offset in map [m],
                     z = the applied heading offset [deg]. For the report only.

The node stays alive until it is stopped (run_inside.sh does that at the end of the run): a transient_local
message exists only while its publisher does, so an estimator started later still receives it. Every value is
logged.

Inside the sim container (ROS 2 sourced):
  python3 -u eval/initial_pose_publisher.py --scenario relocalization     (run_inside.sh starts it like this)
  python3 -u eval/initial_pose_publisher.py --scenario relocalization --offset-m 8 --dir-deg 90 --heading-deg 60
"""
# Options, maths, reproducible randomness, Ctrl-C, paths.
import argparse
import math
import random
import signal
import sys
from pathlib import Path

# ROS 2 and the shared configs.
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

# Repo root (this file is eval/initial_pose_publisher.py); the scenario settings and routes live in sim/.
ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'sim'
sys.path.insert(0, str(SIM / 'config'))
import settings  # noqa: E402  (sim/config/settings.py)


def start_pose(s):
    """The scenario's start: (x, y, z, segment heading [rad], the waypoint's own yaw [deg], index, route file name).
    Same waypoint the .xosc teleports the car to (make_scenarios.py): start_at modulo the route length."""
    route_file = SIM / 'scenarios' / 'routes' / f'{s["route"]}{"_reverse" if s["reverse"] else ""}.yaml'
    route = yaml.safe_load(open(route_file))
    wps, closed = route['waypoints'], route.get('closed', True)
    k = s['start_at'] % len(wps)
    x, y, z, yaw_deg = wps[k]
    # The segment that starts at the waypoint, towards the next one (a closed loop wraps around); the last
    # waypoint of an open route has no next: the segment that ends there.
    if closed or k + 1 < len(wps):
        nx, ny = wps[(k + 1) % len(wps)][:2]
        heading = math.atan2(ny - y, nx - x)
    else:
        px, py = wps[k - 1][:2]
        heading = math.atan2(y - py, x - px)
    return x, y, z, heading, yaw_deg, k, route_file.name


def offset_pose(x, y, heading, offset_m, dir_deg, heading_deg):
    """Start (x, y, heading) -> (x, y, yaw) of the offset pose, plus the applied (dx, dy), all in map."""
    bearing = heading + math.radians(dir_deg)
    dx, dy = offset_m * math.cos(bearing), offset_m * math.sin(bearing)
    return x + dx, y + dy, heading + math.radians(heading_deg), dx, dy


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', required=True, help='the scenario in config/sim.yaml (start pose and offsets)')
    ap.add_argument('--offset-m', type=float, help='position offset [m] (overrides the scenario)')
    ap.add_argument('--dir-deg', help='direction of the position offset relative to the heading, or "random"')
    ap.add_argument('--heading-deg', type=float, help='heading offset [deg] (overrides the scenario)')
    ap.add_argument('--seed', type=int, help='seed for "random" (overrides the scenario)')
    a = ap.parse_args()
    # The offsets: scenario values, then explicit options on top.
    s = settings.load(a.scenario)
    offset_m = a.offset_m if a.offset_m is not None else s.get('init_offset_m')
    dir_deg = a.dir_deg if a.dir_deg is not None else s.get('init_offset_dir_deg', 0.0)
    heading_deg = a.heading_deg if a.heading_deg is not None else s.get('init_heading_offset_deg', 0.0)
    seed = a.seed if a.seed is not None else s.get('seed', 0)
    if offset_m is None:
        raise SystemExit('no initial pose for this scenario (init_offset_m is null): nothing to publish')
    offset_m, heading_deg = float(offset_m), float(heading_deg)
    drawn = ''
    if str(dir_deg) == 'random':
        rng = random.Random(seed)
        dir_deg = rng.uniform(0.0, 360.0)
        heading_deg *= rng.choice((-1.0, 1.0))
        drawn = f' (drawn from seed {seed})'
    dir_deg = float(dir_deg)

    x0, y0, z0, heading, wp_yaw_deg, k, route_name = start_pose(s)
    x, y, yaw, dx, dy = offset_pose(x0, y0, heading, offset_m, dir_deg, heading_deg)

    # run_inside.sh starts us in the background with Ctrl-C ignored: switch it back on BEFORE rclpy.init(), whose
    # own handler then wakes spin() up and passes the signal on (set after init, nothing wakes the blocking wait).
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    node = rclpy.create_node('initial_pose_publisher')
    log = node.get_logger()
    # transient_local depth 1: the one message is kept for subscribers that come later.
    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', latched)
    offset_pub = node.create_publisher(Vector3Stamped, '/eval/init_offset', latched)
    log.info(f'scenario {a.scenario}: start = waypoint {k} of {route_name} ({x0:.2f}, {y0:.2f}, {z0:.2f}), segment '
             f'heading {math.degrees(heading):.1f} deg (the waypoint\'s own yaw: {wp_yaw_deg:.1f} deg); offset '
             f'{offset_m:.2f} m at {dir_deg:.1f} deg{drawn} = dx {dx:+.2f}, dy {dy:+.2f} m; heading {heading_deg:+.1f} deg '
             f'-> /initialpose ({x:.2f}, {y:.2f}, {z0:.2f}, yaw {math.degrees(yaw):.1f} deg); waiting for /clock')

    published = []

    def on_clock(c):
        # Once, stamped with the first simulation time seen (the bridge publishes /clock once the car exists).
        if published:
            return
        published.append(True)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = c.clock
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z0
        # Yaw only (roll = pitch = 0): quaternion about z.
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # Row-major 6x6 (x, y, z, roll, pitch, yaw): the sigmas described in the header, squared.
        sigmas = [offset_m, offset_m, 1.0, math.radians(5), math.radians(5), math.radians(abs(heading_deg))]
        for i, sd in enumerate(sigmas):
            msg.pose.covariance[i * 7] = sd * sd
        pose_pub.publish(msg)
        off = Vector3Stamped()
        off.header = msg.header
        off.vector.x, off.vector.y, off.vector.z = dx, dy, heading_deg
        offset_pub.publish(off)
        log.info(f'/initialpose published at sim time {c.clock.sec + c.clock.nanosec * 1e-9:.2f} s (kept for late '
                 f'subscribers while this node runs)')

    node.create_subscription(Clock, '/clock', on_clock, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
