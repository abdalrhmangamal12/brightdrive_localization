"""Unit tests of the bridge's conversions: time stamps, the sweep rule, the sweep assembler's gap check, the IMU
bias, the GNSS covariance, the `enabled` switch, the camera's JPEG, the synchronous-mode watch and the tick grid. No server needed; carla and rclpy must import, so run inside the sim container:

  docker compose run --rm --no-deps sim python3 -m pytest -q -p no:cacheprovider /ws/src/brightdrive_localization/sim/carla_bridge
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import carla_bridge as cb  # noqa: E402

# A 64-beam LiDAR as in sensors.yaml, for ring_of.
ATTRS = {'channels': 64, 'upper_fov': 22.5, 'lower_fov': -22.5}


def measurement(t, n=3):
    """A stand-in for carla.LidarMeasurement: n points (x, y, z, intensity) at one timestamp."""
    return SimpleNamespace(timestamp=t, raw_data=np.tile(np.array([1.0, 0.0, 0.0, 0.5], np.float32), (n, 1)).tobytes())


def test_ros_time_carries_a_rounded_up_second():
    t = cb.ros_time(2.9999999999999996)
    assert (t.sec, t.nanosec) == (3, 0)


def test_ros_time_plain_values():
    assert (cb.ros_time(0.0).sec, cb.ros_time(0.0).nanosec) == (0, 0)
    assert (cb.ros_time(1.25).sec, cb.ros_time(1.25).nanosec) == (1, 250_000_000)
    t = cb.ros_time(123 * 0.01)   # 1.23 as CARLA accumulates ticks: 1.2300000000000002
    assert (t.sec, t.nanosec) == (1, 230_000_000)


def test_sweep_rule():
    assert cb.sweep_product(20.0, 0.01, 5) == pytest.approx(1.0, abs=1e-12)
    assert cb.sweep_product(20.0, 0.05, 1) == pytest.approx(1.0, abs=1e-12)
    assert cb.sweep_product(10.0, 0.01, 5) == pytest.approx(0.5, abs=1e-12)


def test_assembler_publishes_whole_sweeps():
    out = []
    a = cb.SweepAssembler(5, 0.01, ATTRS, 'lidar', out.append, warn=lambda msg: None)
    for k in range(10):
        a.add(measurement(1.0 + k * 0.01))
    assert len(out) == 2 and a.dropped == 0
    pts = np.frombuffer(bytes(out[0].data), dtype=cb.POINT_DTYPE)
    assert out[0].width == 15 and out[0].point_step == 22
    assert sorted(set(np.round(pts['time'].astype(float), 3))) == pytest.approx([0.0, 0.01, 0.02, 0.03, 0.04])
    assert (out[0].header.stamp.sec, out[0].header.stamp.nanosec) == (1, 0)
    assert (out[1].header.stamp.sec, out[1].header.stamp.nanosec) == (1, 50_000_000)


def test_assembler_drops_a_partial_sweep_on_a_gap():
    out, warned = [], []
    a = cb.SweepAssembler(5, 0.01, ATTRS, 'lidar', out.append, warn=warned.append)
    for k in range(3):
        a.add(measurement(1.0 + k * 0.01))     # 1.00, 1.01, 1.02
    a.add(measurement(1.04))                   # 1.03 is missing: the 3 slices go, 1.04 starts a new sweep
    assert out == [] and a.dropped == 1 and len(warned) == 1 and '20.000 ms' in warned[0]
    for k in range(5, 9):
        a.add(measurement(1.0 + k * 0.01))     # 1.05 .. 1.08: with 1.04 that is 5 slices
    assert len(out) == 1 and a.dropped == 1
    assert (out[0].header.stamp.sec, out[0].header.stamp.nanosec) == (1, 40_000_000)


def test_ring_ends():
    low = np.array([[10.0, 0.0, 10.0 * np.tan(np.radians(-22.5)), 0.0]], np.float32)
    high = np.array([[10.0, 0.0, 10.0 * np.tan(np.radians(22.5)), 0.0]], np.float32)
    assert cb.ring_of(low, ATTRS)[0] == 0 and cb.ring_of(high, ATTRS)[0] == 63


def test_imu_bias_after_axis_flip():
    m = SimpleNamespace(timestamp=1.0, gyroscope=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                        accelerometer=SimpleNamespace(x=1.0, y=2.0, z=3.0))
    msg = cb.imu_msg(m, 'imu', [1e-4] * 3, [1e-2] * 3, accel_bias=(0.1, 0.2, 0.3))
    acc = msg.linear_acceleration
    assert (acc.x, acc.y, acc.z) == pytest.approx((1.1, -1.8, 3.3))
    assert (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) == pytest.approx((-0.1, 0.2, -0.3))
    assert msg.orientation_covariance[0] == -1.0


def test_gnss_covariance_diagonal_known():
    m = SimpleNamespace(timestamp=1.0, latitude=0.001, longitude=0.002, altitude=1.8)
    msg = cb.gnss_msg(m, 'gnss', 0.3)
    assert msg.position_covariance_type == 2   # COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert list(msg.position_covariance) == pytest.approx([0.09, 0, 0, 0, 0.09, 0, 0, 0, 0.09])


def test_active_sensors_skips_disabled():
    cfg = {'sensors': {'lidar': {}, 'imu': {'enabled': True}, 'camera': {'enabled': False}}}
    assert list(cb.active_sensors(cfg)) == ['lidar', 'imu']


def test_image_msg_is_jpeg():
    # An 8x4 BGRA frame: blue on the left half, red on the right half.
    frame = np.zeros((4, 8, 4), np.uint8)
    frame[:, :4] = (255, 0, 0, 255)
    frame[:, 4:] = (0, 0, 255, 255)
    m = SimpleNamespace(timestamp=2.5, width=8, height=4, raw_data=frame.tobytes())
    msg = cb.image_msg(m, 'camera', 90)
    assert msg.format == 'jpeg' and msg.header.frame_id == 'camera'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (2, 500_000_000)
    data = bytes(msg.data)
    assert data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9'   # JPEG start and end markers
    decoded = cb.cv2.imdecode(np.frombuffer(data, np.uint8), cb.cv2.IMREAD_COLOR)
    assert decoded.shape == (4, 8, 3)
    assert decoded[2, 1].argmax() == 0 and decoded[2, 6].argmax() == 2   # BGR: blue left, red right


def test_sync_watch_flags_the_first_differing_delta():
    w = cb.SyncWatch()
    f32 = 0.009999999776482582                     # 0.01 s as CARLA reports it in synchronous mode
    assert w.update(f32, 1.00) is False and w.synchronous
    assert w.update(f32, 1.01) is False and w.synchronous
    assert w.update(0.01, 1.02) is False            # the first asynchronous frame can still carry the old delta
    assert w.update(0.00435, 1.03) is True          # a real frame time: not synchronous
    assert not w.synchronous and w.lost == (0.00435, 1.03)
    assert w.update(0.0052, 1.04) is False          # reported once


class FakeSensor:
    """What stop_sensors / remove_sensors use of a CARLA sensor: is_alive (property), is_listening() (method)."""

    def __init__(self):
        self.is_alive, self.listening, self.stops, self.destroyed = True, True, 0, False

    def is_listening(self):
        return self.listening

    def stop(self):
        assert self.listening, 'stop() on a sensor that is not listening (CARLA prints a warning)'
        self.listening, self.stops = False, self.stops + 1

    def destroy(self):
        self.destroyed = True


class FakeWorld:
    def remove_on_tick(self, tick_id):
        pass


def test_a_stopped_sensor_is_not_stopped_again_when_removed():
    s, sensors, ticks = FakeSensor(), [], [7]
    sensors.append(s)
    cb.stop_sensors(sensors)                      # the world turned asynchronous
    cb.remove_sensors(FakeWorld(), sensors, ticks)   # then the car went
    assert s.stops == 1 and s.destroyed and not s.listening and sensors == [] and ticks == []


def test_tick_grid_accepts_only_whole_ticks_after_the_previous_measurement():
    g, f32, t = cb.TickGrid(0.01), 0.009999999776482582, 1000.0
    assert g.accept(t)                                   # the first measurement sets the grid
    for _ in range(5):                                   # IMU: every tick, float32 steps
        t += f32
        assert g.accept(t)
    assert g.accept(t + 0.11) and g.accept(t + 0.21)     # GNSS: 11 ticks (first fix), then 10
    t += 0.21
    assert not g.accept(t + 0.00454)                     # the first asynchronous frame (4.54 ms measured)
    assert not g.accept(t + 0.00454 + 0.0047)            # and the next one
    assert g.accept(t + 0.00454 + 0.0047 + 0.01)         # relative to the previous: a tick later is on grid again
    assert g.gap_s == pytest.approx(0.01)
