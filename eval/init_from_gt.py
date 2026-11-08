#!/usr/bin/env python3
"""An /initialpose made from ground truth plus a known offset, for initialization tests on a replayed bag.

The relocalization scenario's /initialpose (eval/initial_pose_publisher.py) is the route's start waypoint plus an
offset, so its true error is only known up to the car settling on the road. Here the pose is the ground truth
(/eval/ground_truth, base_link in map) at a chosen time plus the offset, so the error handed to the localizer is
exact. Ground truth is used here only; the localizer sees nothing but /initialpose.

  --at S          use the first ground-truth message at least S s after the first one (the car must be standing
                  then and stay still until the localizer searches: it uses an /initialpose position only when the
                  IMU says the car has not moved since the stamp)
  --offset-m M    position offset [m] along --dir-deg (relative to the true heading: 0 = ahead, 90 = left)
  --heading-deg H heading offset [deg]
  --dz Z          added to the true z [m]; --click-z sets z = 0 like a 2-D pose estimate click in RViz / Foxglove

  /initialpose       geometry_msgs/PoseWithCovarianceStamped, frame map, stamped with the ground-truth time, once,
                     reliable + transient_local depth 1; covariance sigma_xy = max(M, 0.5) m, sigma_z 1 m,
                     sigma_roll = sigma_pitch = 5 deg, sigma_yaw = max(|H|, 5) deg
  /eval/init_offset  geometry_msgs/Vector3Stamped, same QoS: x, y = applied position offset in map [m], z = H [deg]
                     (eval/loc_metrics.py reads it for the convergence of an initialization run)

The node stays up until it is stopped (a transient_local message exists only while its publisher does). Run it in
the loc container beside localization_replay.launch.py; eval/run_init_tests.sh does that.
"""
import argparse
import math
import signal
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from initial_pose_publisher import offset_pose  # noqa: E402


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--at', type=float, default=2.5)
    ap.add_argument('--offset-m', type=float, default=1.0)
    ap.add_argument('--dir-deg', type=float, default=45.0)
    ap.add_argument('--heading-deg', type=float, default=10.0)
    ap.add_argument('--dz', type=float, default=0.0)
    ap.add_argument('--click-z', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('init_from_gt', parameter_overrides=[Parameter('use_sim_time', value=True)])
    log = node.get_logger()
    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', latched)
    off_pub = node.create_publisher(Vector3Stamped, '/eval/init_offset', latched)
    state = {'t_first': None, 'sent': False}

    def on_gt(m):
        if state['sent']:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if state['t_first'] is None:
            state['t_first'] = t
        if t - state['t_first'] < a.at:
            return
        p, q = m.pose.pose.position, m.pose.pose.orientation
        heading = yaw_of(q)
        x, y, yaw, dx, dy = offset_pose(p.x, p.y, heading, a.offset_m, a.dir_deg, a.heading_deg)
        z = 0.0 if a.click_z else p.z + a.dz
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = m.header.stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
        # Roll and pitch are not known to a user either: level.
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        sxy, syaw = max(a.offset_m, 0.5), math.radians(max(abs(a.heading_deg), 5.0))
        for i, s in enumerate([sxy, sxy, 1.0, math.radians(5.0), math.radians(5.0), syaw]):
            msg.pose.covariance[i * 7] = s * s
        pose_pub.publish(msg)
        off = Vector3Stamped()
        off.header = msg.header
        off.vector.x, off.vector.y, off.vector.z = dx, dy, a.heading_deg
        off_pub.publish(off)
        state['sent'] = True
        log.info(f'/initialpose at t {t:.3f} s ({t - state["t_first"]:.2f} s after the first ground truth): '
                 f'truth ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) yaw {math.degrees(heading):.2f} deg, speed '
                 f'{math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y):.3f} m/s -> sent ({x:.3f}, {y:.3f}, '
                 f'{z:.3f}) yaw {math.degrees(yaw):.2f} deg (offset {a.offset_m} m at {a.dir_deg} deg = '
                 f'({dx:.3f}, {dy:.3f}), heading {a.heading_deg:+.1f} deg, z {"0 (click)" if a.click_z else f"{a.dz:+.1f}"})')

    node.create_subscription(Odometry, '/eval/ground_truth', on_gt, 50)
    log.info(f'waiting for /eval/ground_truth: --at {a.at} --offset-m {a.offset_m} --dir-deg {a.dir_deg} '
             f'--heading-deg {a.heading_deg} --dz {a.dz} --click-z {a.click_z}')
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
