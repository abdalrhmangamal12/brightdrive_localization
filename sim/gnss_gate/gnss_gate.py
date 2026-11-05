#!/usr/bin/env python3
"""The GNSS outage, made at the ROS 2 input: "GNSS loss may be emulated by reproducibly withholding the CARLA GNSS
messages at the ROS 2 input".

Subscribes the bridge's /gnss/fix, republishes it as /gnss/fix_gated, and drops every message while

    start_s <= t - t0 < start_s + duration_s      t = the message's stamp (sim time), t0 = the first message's stamp

The estimator subscribes /gnss/fix_gated in EVERY scenario, so this gate always runs; with duration 0 (the
default) it passes everything through. Topic names and QoS come from config/sensors.yaml (gnss: topic,
gated_topic, reliability, queue_depth); the window comes from config/sim.yaml (gnss_outage_start_s,
gnss_outage_duration_s, through config/settings.py). /eval/gnss_outage (diagnostic_msgs/DiagnosticArray, one per
input message) carries the gate's state for plots: level OK = passing, WARN = outage; values t_rel_s, start_s,
end_s, t0_s.

Inside the sim container (ROS 2 sourced):
  python3 -u sim/gnss_gate/gnss_gate.py --scenario gnss_outage      (run_inside.sh starts it like this)
  python3 -u sim/gnss_gate/gnss_gate.py --start 60 --duration 45    (explicit window, for a quick test)
"""
# Command-line options, Ctrl-C, file paths.
import argparse
import signal
import sys
from pathlib import Path

# ROS 2 (Humble, sim container).
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

# sim/ folder (this file is sim/gnss_gate/gnss_gate.py); the scenario settings live in config/.
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / 'config'))
import settings  # noqa: E402  (sim/config/settings.py)


class GnssGate:
    """Holds the window and the two publishers; `on_fix` is the only callback."""

    def __init__(self, node, gnss_cfg, start_s, duration_s):
        self.node, self.log = node, node.get_logger()
        self.start_s, self.end_s = start_s, start_s + duration_s
        # t0 = stamp of the first message; None until it arrives.
        self.t0 = None
        # Was the previous message inside the window? For the two log lines.
        self.was_out = False
        # Same QoS as the bridge's /gnss/fix, so subscribers of the gated topic ask for what they asked before.
        reliability = ReliabilityPolicy.RELIABLE if gnss_cfg['reliability'] == 'reliable' else ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(reliability=reliability, history=HistoryPolicy.KEEP_LAST, depth=gnss_cfg['queue_depth'])
        self.pub = node.create_publisher(NavSatFix, gnss_cfg['gated_topic'], qos)
        self.state_pub = node.create_publisher(DiagnosticArray, '/eval/gnss_outage', 10)
        self.sub = node.create_subscription(NavSatFix, gnss_cfg['topic'], self.on_fix, qos)
        if duration_s > 0:
            self.log.info(f'{gnss_cfg["topic"]} -> {gnss_cfg["gated_topic"]}: outage from {start_s:.1f} s to '
                          f'{self.end_s:.1f} s after the first fix')
        else:
            self.log.info(f'{gnss_cfg["topic"]} -> {gnss_cfg["gated_topic"]}: no outage (pass-through)')

    def on_fix(self, msg):
        # Time since the first fix, from the message stamps (sim time, so the window is reproducible).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        t_rel = t - self.t0
        out = self.start_s <= t_rel < self.end_s
        # One line at each edge of the window.
        if out and not self.was_out:
            self.log.info(f'GNSS outage START at t_rel {t_rel:.2f} s (stamp {t:.2f})')
        elif self.was_out and not out:
            self.log.info(f'GNSS outage END at t_rel {t_rel:.2f} s (stamp {t:.2f})')
        self.was_out = out
        # The gate itself.
        if not out:
            self.pub.publish(msg)
        # The state, for plots (eval namespace: the estimator never reads it).
        status = DiagnosticStatus(level=DiagnosticStatus.WARN if out else DiagnosticStatus.OK, name='gnss_gate',
                                  message='outage' if out else 'pass-through',
                                  values=[KeyValue(key='t_rel_s', value=f'{t_rel:.3f}'),
                                          KeyValue(key='start_s', value=f'{self.start_s:.3f}'),
                                          KeyValue(key='end_s', value=f'{self.end_s:.3f}'),
                                          KeyValue(key='t0_s', value=f'{self.t0:.3f}')])
        state = DiagnosticArray()
        state.header.stamp = msg.header.stamp
        state.status = [status]
        self.state_pub.publish(state)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', help='take the window from this scenario in config/sim.yaml')
    ap.add_argument('--start', type=float, help='outage start, seconds after the first fix (overrides the scenario)')
    ap.add_argument('--duration', type=float, help='outage length in seconds, 0 = none (overrides the scenario)')
    a = ap.parse_args()
    # The window: scenario values, then explicit options on top.
    start_s, duration_s = 0.0, 0.0
    if a.scenario:
        s = settings.load(a.scenario)
        start_s, duration_s = float(s['gnss_outage_start_s']), float(s['gnss_outage_duration_s'])
    if a.start is not None:
        start_s = a.start
    if a.duration is not None:
        duration_s = a.duration
    gnss_cfg = yaml.safe_load(open(SIM / 'config' / 'sensors.yaml'))['sensors']['gnss']
    # run_inside.sh starts us in the background with Ctrl-C ignored: switch it back on.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    node = rclpy.create_node('gnss_gate')
    GnssGate(node, gnss_cfg, start_s, duration_s)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
