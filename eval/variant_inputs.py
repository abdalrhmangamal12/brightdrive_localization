#!/usr/bin/env python3
"""Inputs for unseen scenario variants without touching sim/: a GNSS outage of any length and an initial pose with
any offset, on their own topics, so the localizer can be pointed at them by parameters alone.

  --outage-start S --outage-duration D   /gnss/fix (raw, from the bridge) -> /gnss/fix_variant, withheld for D s
                                         starting S s after the first fix; the state on /eval/gnss_outage_variant
                                         (same message as the scenario's gate: level WARN = outage)
  --init-scenario NAME --init-offset-m M --init-dir-deg A --init-heading-deg H
                                         /initialpose_variant (+ /eval/init_offset_variant) at the first /clock:
                                         the scenario's start waypoint + the offsets (eval/initial_pose_publisher.py's
                                         own start_pose / offset_pose), transient local

Ground truth is not used. Run beside a live scenario (eval/run_variant.sh starts and stops it).
"""
import argparse
import math
import sys
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix

sys.path.insert(0, str(Path(__file__).resolve().parent))
import initial_pose_publisher as ipp  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--outage-start', type=float)
    ap.add_argument('--outage-duration', type=float)
    ap.add_argument('--init-scenario')
    ap.add_argument('--init-offset-m', type=float, default=5.0)
    ap.add_argument('--init-dir-deg', type=float, default=45.0)
    ap.add_argument('--init-heading-deg', type=float, default=30.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('variant_inputs')
    log = node.get_logger()
    sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

    if a.outage_start is not None and a.outage_duration is not None:
        pub = node.create_publisher(NavSatFix, '/gnss/fix_variant', sensor)
        state_pub = node.create_publisher(DiagnosticArray, '/eval/gnss_outage_variant', 10)
        t0 = []

        def on_fix(msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not t0:
                t0.append(t)
            t_rel = t - t0[0]
            out = a.outage_start <= t_rel < a.outage_start + a.outage_duration
            if not out:
                pub.publish(msg)
            st = DiagnosticStatus(level=DiagnosticStatus.WARN if out else DiagnosticStatus.OK, name='gnss_gate_variant',
                                  message='outage' if out else 'pass-through',
                                  values=[KeyValue(key='t_rel_s', value=f'{t_rel:.3f}')])
            arr = DiagnosticArray()
            arr.header.stamp = msg.header.stamp
            arr.status = [st]
            state_pub.publish(arr)
        node.create_subscription(NavSatFix, '/gnss/fix', on_fix, sensor)
        log.info(f'/gnss/fix -> /gnss/fix_variant, outage {a.outage_start:.1f} s + {a.outage_duration:.1f} s')

    if a.init_scenario:
        s = ipp.settings.load(a.init_scenario)
        x0, y0, z0, heading, _, k, route = ipp.start_pose(s)
        x, y, yaw, dx, dy = ipp.offset_pose(x0, y0, heading, a.init_offset_m, a.init_dir_deg, a.init_heading_deg)
        pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose_variant', latched)
        off_pub = node.create_publisher(Vector3Stamped, '/eval/init_offset_variant', latched)
        done = []

        def on_clock(c):
            if done:
                return
            done.append(True)
            m = PoseWithCovarianceStamped()
            m.header.stamp = c.clock
            m.header.frame_id = 'map'
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, z0
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            for i, sd in enumerate([a.init_offset_m, a.init_offset_m, 1.0, math.radians(5), math.radians(5),
                                    math.radians(max(abs(a.init_heading_deg), 1.0))]):
                m.pose.covariance[i * 7] = sd * sd
            pose_pub.publish(m)
            o = Vector3Stamped()
            o.header = m.header
            o.vector.x, o.vector.y, o.vector.z = dx, dy, a.init_heading_deg
            off_pub.publish(o)
            log.info(f'/initialpose_variant: waypoint {k} of {route} + {a.init_offset_m} m at {a.init_dir_deg} deg, '
                     f'heading {a.init_heading_deg:+.1f} deg')
        node.create_subscription(Clock, '/clock', on_clock, 10)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
