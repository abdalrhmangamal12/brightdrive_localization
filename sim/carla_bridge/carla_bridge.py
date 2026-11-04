#!/usr/bin/env python3
"""The bridge between CARLA and ROS 2.

It finds our car (the role_name in config/sensors.yaml), attaches the sensors from that file, and publishes every
measurement as a ROS 2 message: LiDAR, IMU, GNSS, /clock and /tf_static. It never spawns the car and never ticks
the world: ScenarioRunner does that. It exits on Ctrl-C / SIGTERM (exit 0), and with exit 1 when the server is
lost: an RPC that fails, or no tick for WATCHDOG_S seconds while a short probe of the server fails too (a server
that answers but does not tick is only reported, once, and the wait goes on).

Inside the sim container (ROS 2 sourced):
  python3 -u /ws/src/brightdrive_localization/sim/carla_bridge/carla_bridge.py [--scenario nominal]

--scenario NAME takes that scenario's GNSS noise from config/sim.yaml (the task sets GNSS quality through
CARLA's sensor noise attributes); without it, the values in config/sensors.yaml are used. The NavSatFix
covariance is diag(sigma^2) of that noise in metres, so the estimator's measurement noise follows the run.
"""
# Standard library: command-line options, stopping on Ctrl-C, wait loops, file paths.
import argparse
import signal
import sys
import time
from pathlib import Path

# numpy: turns the LiDAR's raw bytes into an array we can edit (flip y).
import numpy as np
# CARLA's Python API: talk to the server.
import carla
# ROS 2 for Python (Humble, inside the sim container).
import rclpy
# Reads config/sensors.yaml.
import yaml
# ROS message types we publish.
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
# QoS = delivery rules of a topic (best effort or reliable, how many messages to keep).
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
# Publishes the fixed sensor positions on /tf_static once, kept for late subscribers.
from tf2_ros import StaticTransformBroadcaster

# sim/ folder (this file is sim/carla_bridge/carla_bridge.py); the scenario settings live in config/.
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / 'config'))
import settings  # noqa: E402  (sim/config/settings.py)
# Metres per degree of latitude (and of longitude near the equator, where CARLA towns sit).
M_PER_DEG = 111320.0
# RPC timeout, seconds: also how long a dead server takes to be noticed by an RPC.
SERVER_TIMEOUT_S = 10.0
# Watchdog: no tick for this long (wall seconds) -> one short probe of the server; no answer = the server is lost.
WATCHDOG_S = 10.0
PROBE_TIMEOUT_S = 2.0
# Two consecutive LiDAR slices must be exactly one tick apart; a difference beyond this (float noise) is a gap.
SLICE_GAP_TOL_S = 1e-6


class ServerLost(RuntimeError):
    """The CARLA server stopped answering."""


def server_answers(client):
    """True if the server answers a short RPC. The normal timeout is restored afterwards."""
    client.set_timeout(PROBE_TIMEOUT_S)
    try:
        client.get_server_version()
        return True
    except RuntimeError:
        return False
    finally:
        client.set_timeout(SERVER_TIMEOUT_S)


def ros_time(seconds):
    """Simulation seconds (a float from CARLA) -> ROS Time (whole seconds + nanoseconds).
    Whole nanoseconds first, then the split: 2.9999999999999996 s is 3 s + 0 ns. (Splitting the float first gave
    sec 2 and a remainder that rounded up to 1e9 ns, which wrapped to 0 without the carry: 2.0 s.)"""
    sec, nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def quaternion(roll_deg, pitch_deg, yaw_deg):
    """Roll, pitch, yaw in degrees -> quaternion (x, y, z, w), the rotation format ROS messages use."""
    # Half angles in radians.
    r, p, y = (np.radians(a) / 2.0 for a in (roll_deg, pitch_deg, yaw_deg))
    # Standard roll-pitch-yaw (Z-Y-X) to quaternion formula.
    return (np.sin(r) * np.cos(p) * np.cos(y) - np.cos(r) * np.sin(p) * np.sin(y),
            np.cos(r) * np.sin(p) * np.cos(y) + np.sin(r) * np.cos(p) * np.sin(y),
            np.cos(r) * np.cos(p) * np.sin(y) - np.sin(r) * np.sin(p) * np.cos(y),
            np.cos(r) * np.cos(p) * np.cos(y) + np.sin(r) * np.sin(p) * np.sin(y))


def ros_to_carla(xyz, rpy):
    """Mount pose from the YAML (ROS: y left) -> CARLA (y right): flip y, pitch and yaw."""
    # Position and angles as written in the YAML.
    x, y, z = xyz
    roll, pitch, yaw = rpy
    # Same pose in CARLA's axes.
    return carla.Transform(carla.Location(x=x, y=-y, z=z), carla.Rotation(roll=roll, pitch=-pitch, yaw=-yaw))


# One LiDAR point as published: x, y, z, intensity (float32), time (float32: seconds after the sweep's
# header.stamp), ring (uint16: beam index, 0 = the lowest beam). 22 bytes, no padding. These are the fields
# FAST-LIO2's Velodyne handler (lidar_type 2, timestamp_unit 0) reads, and what a deskew step needs.
POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u2')])
POINT_FIELDS = [PointField(name=n, offset=POINT_DTYPE.fields[n][1],
                           datatype=PointField.UINT16 if n == 'ring' else PointField.FLOAT32, count=1)
                for n in POINT_DTYPE.names]


def slice_points(m):
    """carla.LidarMeasurement -> (N, 4) float32 x, y, z, intensity in ROS axes."""
    # CARLA gives 4 float32 per point: x, y, z, intensity; copy so we can change it.
    points = np.frombuffer(m.raw_data, dtype=np.float32).reshape(-1, 4).copy()
    # CARLA's y points right, ROS's y points left.
    points[:, 1] *= -1.0
    return points


def ring_of(points, attrs):
    """Beam index of each point from its elevation: CARLA spaces the `channels` beams evenly from lower_fov to
    upper_fov. Ring 0 = the lowest beam (Velodyne convention), channels - 1 = the highest."""
    elev = np.degrees(np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1])))
    step = (float(attrs['upper_fov']) - float(attrs['lower_fov'])) / (int(attrs['channels']) - 1)
    ring = np.rint((elev - float(attrs['lower_fov'])) / step)
    return np.clip(ring, 0, int(attrs['channels']) - 1).astype(np.uint16)


def cloud_msg(stamp, frame_id, slices, attrs):
    """[(timestamp, (N, 4) points), ...] of one sweep -> sensor_msgs/PointCloud2 with the fields above.
    `time` of a point = its slice's timestamp - the first slice's timestamp (CARLA measures a whole slice at
    one instant, so every point of a slice has the same time)."""
    n = sum(len(pts) for _, pts in slices)
    cloud = np.empty(n, dtype=POINT_DTYPE)
    k = 0
    for t, pts in slices:
        cloud['x'][k:k + len(pts)], cloud['y'][k:k + len(pts)], cloud['z'][k:k + len(pts)] = pts[:, 0], pts[:, 1], pts[:, 2]
        cloud['intensity'][k:k + len(pts)] = pts[:, 3]
        cloud['time'][k:k + len(pts)] = t - stamp
        cloud['ring'][k:k + len(pts)] = ring_of(pts, attrs)
        k += len(pts)
    msg = PointCloud2()
    # Stamp = simulation time of the first slice; frame = where the points are measured.
    msg.header.stamp = ros_time(stamp)
    msg.header.frame_id = frame_id
    # An unordered cloud: 1 row of N points.
    msg.height = 1
    msg.width = n
    msg.fields = POINT_FIELDS
    # x86 is little-endian.
    msg.is_bigendian = False
    msg.point_step = POINT_DTYPE.itemsize
    msg.row_step = POINT_DTYPE.itemsize * n
    # CARLA returns no invalid (NaN) points.
    msg.is_dense = True
    # The points themselves, as raw bytes.
    msg.data = cloud.tobytes()
    return msg


def sweep_product(rotation_frequency, tick_s, assemble_sweeps):
    """rotation_frequency x tick_s x assemble_sweeps: exactly 1 when `assemble_sweeps` ticks are one revolution."""
    return float(rotation_frequency) * float(tick_s) * int(assemble_sweeps)


class SweepAssembler:
    """Turns CARLA's per-tick LiDAR measurements into one message per revolution.

    At a world tick shorter than the revolution (tick 0.01 s, rotation_frequency 20 Hz) CARLA delivers a 72 deg
    slice per tick, one measurement per tick, never more or fewer (synchronous mode). `assemble_sweeps`
    (sensors.yaml) = how many slices make a revolution (5): every n consecutive measurements are one sweep.
    The sweep's first slice is wherever the revolution stood when the bridge attached (the sector boundaries
    are fixed at 72 deg steps); that phase stays the same for the whole run and does not matter to a deskew
    step or a matcher, which use the per-point `time`. (Finding the 0-deg crossing from the points instead was
    tried and is not reliable: a sector with few returns, e.g. open space, shifts its mean azimuth.)
    With assemble_sweeps <= 1 (tick 0.05 s: a whole sweep per tick) every measurement is published as it is,
    with time 0 for every point.
    Consecutive slices must be exactly one tick apart (CARLA delivers one per tick); a slice that arrives later
    than that means one is missing, and the sweep being built is thrown away: a published sweep never has a
    hole. `dropped` counts those."""

    def __init__(self, n, tick_s, attrs, frame_id, publish, warn=print):
        self.n, self.tick_s, self.attrs, self.frame_id = max(int(n), 1), float(tick_s), attrs, frame_id
        self.publish, self.warn = publish, warn
        # Slices of the revolution being built.
        self.slices = []
        self.dropped = 0

    def add(self, m):
        if self.slices:
            gap = m.timestamp - self.slices[-1][0]
            if abs(gap - self.tick_s) > SLICE_GAP_TOL_S:
                self.dropped += 1
                self.warn(f'LiDAR: slices {gap * 1e3:.3f} ms apart, not {self.tick_s * 1e3:g} ms: dropping a partial '
                          f'sweep of {len(self.slices)} slice(s); dropped so far: {self.dropped}')
                self.slices = []
        self.slices.append((m.timestamp, slice_points(m)))
        if len(self.slices) >= self.n:
            self.publish(cloud_msg(self.slices[0][0], self.frame_id, self.slices, self.attrs))
            self.slices = []


def imu_msg(m, frame_id, gyro_var, accel_var, accel_bias=(0.0, 0.0, 0.0)):
    """carla.IMUMeasurement -> sensor_msgs/Imu. `accel_bias` (m/s^2, ROS axes) is added after the axis flip."""
    msg = Imu()
    # Stamp and frame, as for the LiDAR.
    msg.header.stamp = ros_time(m.timestamp)
    msg.header.frame_id = frame_id
    # CARLA's IMU gives no orientation: -1 in the first covariance entry means "not provided" (ROS convention).
    msg.orientation_covariance[0] = -1.0
    # Turn rate, rad/s. Flipping the y axis flips the sign of rotations about x and z (left- to right-handed).
    msg.angular_velocity.x = -m.gyroscope.x
    msg.angular_velocity.y = m.gyroscope.y
    msg.angular_velocity.z = -m.gyroscope.z
    # Acceleration, m/s^2, gravity included. Only the y component flips. Then the bridge's own constant bias
    # (sensors.yaml bridge_accel_bias, already in ROS axes): CARLA's IMU has no accelerometer bias.
    msg.linear_acceleration.x = m.accelerometer.x + accel_bias[0]
    msg.linear_acceleration.y = -m.accelerometer.y + accel_bias[1]
    msg.linear_acceleration.z = m.accelerometer.z + accel_bias[2]
    # Noise variances from the YAML (stddev squared) on the diagonals of the 3x3 covariances.
    msg.angular_velocity_covariance[0::4] = gyro_var
    msg.linear_acceleration_covariance[0::4] = accel_var
    return msg


def gnss_msg(m, frame_id, sigma_m):
    """carla.GnssMeasurement -> sensor_msgs/NavSatFix with covariance diag(sigma_m^2)."""
    msg = NavSatFix()
    # Stamp and frame.
    msg.header.stamp = ros_time(m.timestamp)
    msg.header.frame_id = frame_id
    # Always set the status: its default differs between Humble (0 = FIX) and Jazzy (-2 = UNKNOWN).
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    # Degrees, degrees, metres.
    msg.latitude = m.latitude
    msg.longitude = m.longitude
    msg.altitude = m.altitude
    # The noise the sensor was given (metres, the same on every axis) as a diagonal covariance, m^2.
    msg.position_covariance[0::4] = [sigma_m ** 2] * 3
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def static_transforms(cfg, base_frame='base_link'):
    """base_link -> each sensor frame, from the YAML (already in ROS axes)."""
    out = []
    for s in cfg['sensors'].values():
        t = TransformStamped()
        # Parent = the car, child = the sensor.
        t.header.frame_id = base_frame
        t.child_frame_id = s['frame_id']
        # Where the sensor sits on the car (metres).
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, s['xyz'])
        # How it is turned (degrees -> quaternion).
        q = quaternion(*s['rpy'])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
        out.append(t)
    return out


def find_hero(client, role_name):
    """Wait until a vehicle with our role_name exists; return (world, car)."""
    while True:
        # Ask again each time: a scenario may reload the town (a new world).
        world = client.get_world()
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == role_name:
                return world, actor
        time.sleep(0.5)


def remove_sensors(world, sensors, tick_ids):
    """Remove our /clock callback and our sensors (sensors are NOT destroyed with their car).
    Each one leaves its list before it is removed, so a second call (car gone, then Ctrl-C) does nothing twice."""
    while tick_ids:
        world.remove_on_tick(tick_ids.pop())
    while sensors:
        sensor = sensors.pop()
        # It may be gone already if the run was stopped half-way through this loop.
        if sensor.is_alive:
            sensor.stop()
            sensor.destroy()


def apply_scenario_gnss(cfg, scenario):
    """Overwrite the GNSS noise in the sensor config with the scenario's (metres -> CARLA's degrees)."""
    # The scenario's settings = defaults + its own changes (sim/config/settings.py).
    s = settings.load(scenario)
    attrs = cfg['sensors']['gnss']['attributes']
    # Random error and constant offset, the same on latitude and longitude; altitude gets the same in metres.
    attrs['noise_lat_stddev'] = attrs['noise_lon_stddev'] = s['gnss_noise_m'] / M_PER_DEG
    attrs['noise_lat_bias'] = attrs['noise_lon_bias'] = s['gnss_bias_m'] / M_PER_DEG
    attrs['noise_alt_stddev'], attrs['noise_alt_bias'] = s['gnss_noise_m'], s['gnss_bias_m']
    return s['gnss_noise_m'], s['gnss_bias_m']


def main():
    # Optional: which scenario's GNSS noise to use.
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', help='a scenario name from config/sim.yaml')
    a = ap.parse_args()
    # Read the car and sensor description.
    cfg = yaml.safe_load(open(SIM / 'config' / 'sensors.yaml'))
    if a.scenario:
        noise, bias = apply_scenario_gnss(cfg, a.scenario)
    # GNSS sigma in metres for the covariance: apply_scenario_gnss sets the altitude stddev to the scenario's
    # gnss_noise_m; without --scenario it is the sensor file's value.
    gnss_sigma_m = float(cfg['sensors']['gnss']['attributes']['noise_alt_stddev'])
    # The sweep rule must hold, or `assemble_sweeps` slices are not one revolution: refuse to start.
    lidar = cfg['sensors']['lidar']
    tick_s = float(settings.config()['tick_s'])
    n_slices = int(lidar.get('assemble_sweeps', 1))
    product = sweep_product(lidar['attributes']['rotation_frequency'], tick_s, n_slices)
    if abs(product - 1.0) > 1e-9:
        raise SystemExit(f'sensors.yaml / sim.yaml: rotation_frequency x tick_s x assemble_sweeps = '
                         f'{lidar["attributes"]["rotation_frequency"]} x {tick_s} x {n_slices} = {product:g}, not 1: '
                         f'{n_slices} slices are not one revolution')
    # The bridge's own accelerometer bias (ROS axes), 0 unless the sensor file says otherwise.
    accel_bias = [float(v) for v in cfg['sensors']['imu'].get('bridge_accel_bias', [0.0, 0.0, 0.0])]
    # Start ROS 2 and create this node. Ctrl-C is left to Python (a KeyboardInterrupt, handled at the end): if ROS
    # shut itself down first, CARLA's callbacks would keep publishing into a closed ROS ("context is invalid").
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # The run script starts us in the background, and bash starts background jobs with Ctrl-C ignored: switch it
    # back on, and treat a stop request (SIGTERM, e.g. docker stop) the same way.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = rclpy.create_node('carla_bridge')
    log = node.get_logger()
    if a.scenario:
        log.info(f'scenario {a.scenario}: GNSS noise {noise} m, bias {bias} m')
    log.info(f'GNSS covariance diag({gnss_sigma_m ** 2:g}) m^2; IMU bridge_accel_bias {accel_bias} m/s^2 '
             f'(ROS axes, added after the axis conversion)')

    # One publisher per sensor: topic, queue depth and delivery (best_effort / reliable) from the YAML.
    pubs = {}
    for name, s in cfg['sensors'].items():
        reliability = ReliabilityPolicy.RELIABLE if s['reliability'] == 'reliable' else ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(reliability=reliability, history=HistoryPolicy.KEEP_LAST, depth=s['queue_depth'])
        msg_type = {'lidar': PointCloud2, 'imu': Imu, 'gnss': NavSatFix}[name]
        pubs[name] = node.create_publisher(msg_type, s['topic'], qos)
    # /clock: simulation time for every node that runs with use_sim_time.
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    # Sensor positions on the car, published once (latched).
    tf_static = StaticTransformBroadcaster(node)
    tf_static.sendTransform(static_transforms(cfg))

    # IMU noise variances (stddev^2), copied into every Imu message.
    imu_attrs = cfg['sensors']['imu']['attributes']
    gyro_var = [imu_attrs[f'noise_gyro_stddev_{a}'] ** 2 for a in 'xyz']
    accel_var = [imu_attrs[f'noise_accel_stddev_{a}'] ** 2 for a in 'xyz']

    # Connect to the CARLA server.
    client = carla.Client('localhost', 2000)
    client.set_timeout(SERVER_TIMEOUT_S)
    # What we add to the world, kept so we can remove it: sensors and the id of our /clock callback.
    sensors, tick_ids, world = [], [], None
    code = 0
    try:
        while rclpy.ok():
            log.info(f'waiting for a vehicle with role_name={cfg["hero"]["role_name"]} ...')
            world, hero = find_hero(client, cfg['hero']['role_name'])
            log.info(f'found {hero.type_id} (id {hero.id}); attaching {", ".join(cfg["sensors"])}')

            # /clock: CARLA calls this function once per tick with the new simulation time. It also notes the
            # wall time of the tick for the watchdog below.
            last_tick = [time.monotonic()]

            def on_tick(snap):
                clock_pub.publish(Clock(clock=ros_time(snap.timestamp.elapsed_seconds)))
                last_tick[0] = time.monotonic()
            tick_ids.append(world.on_tick(on_tick))

            # Spawn each sensor on the car and connect it to its converter + publisher.
            lib = world.get_blueprint_library()
            for name, s in cfg['sensors'].items():
                bp = lib.find(s['blueprint'])
                for key, value in s['attributes'].items():
                    bp.set_attribute(key, str(value))
                sensor = world.spawn_actor(bp, ros_to_carla(s['xyz'], s['rpy']), attach_to=hero)
                frame, pub = s['frame_id'], pubs[name]
                # The callback runs for every measurement: convert it, publish it.
                if name == 'lidar':
                    # One message per revolution: `assemble_sweeps` slices (1 = every measurement as it is).
                    assembler = SweepAssembler(n_slices, tick_s, s['attributes'], frame, pub.publish, log.warning)
                    log.info(f'LiDAR: {n_slices} slice(s) per published sweep '
                             f'(tick {tick_s} s x {n_slices} = one revolution at '
                             f'{s["attributes"]["rotation_frequency"]} Hz); fields x y z intensity time ring')
                    sensor.listen(assembler.add)
                elif name == 'imu':
                    sensor.listen(lambda m, f=frame, p=pub: p.publish(imu_msg(m, f, gyro_var, accel_var, accel_bias)))
                else:
                    sensor.listen(lambda m, f=frame, p=pub: p.publish(gnss_msg(m, f, gnss_sigma_m)))
                sensors.append(sensor)

            # Nothing else to do while the car exists: the callbacks publish. Check twice a second if it is gone
            # (a scenario ends by destroying it). world.get_actors() reads the client's copy of the world state,
            # which every tick refreshes, not the server: it sees the car go after the next tick, but it never
            # sees a dead server. The watchdog does: no tick for WATCHDOG_S -> probe the server.
            quiet = False
            while rclpy.ok() and hero.id in [a.id for a in world.get_actors().filter('vehicle.*')]:
                time.sleep(0.5)
                if time.monotonic() - last_tick[0] < WATCHDOG_S:
                    quiet = False
                elif not server_answers(client):
                    raise ServerLost(f'no tick for {WATCHDOG_S:g} s and no answer within {PROBE_TIMEOUT_S:g} s')
                elif not quiet:
                    log.warning(f'server alive, not ticking (no tick for {WATCHDOG_S:g} s); waiting')
                    quiet = True
            log.info('the car is gone; removing our sensors')
            remove_sensors(world, sensors, tick_ids)
    except KeyboardInterrupt:
        pass
    except ServerLost as err:
        code = 1
        log.error(f'server lost ({err}); exiting')
        world = None   # nothing left to remove
    except RuntimeError as err:
        # The CARLA client raises RuntimeError when the server stops answering (crash, docker stop) and for other
        # failures (a sensor that cannot spawn, ...). A short probe tells the two apart; both end the run.
        code = 1
        if server_answers(client):
            log.error(f'CARLA error, exiting: {err}')
        else:
            log.error(f'server lost ({err}); exiting')
            world = None   # nothing left to remove
    finally:
        # Ctrl-C: the same clean-up (CARLA callbacks first), then stop ROS.
        if world is not None:
            try:
                remove_sensors(world, sensors, tick_ids)
            except RuntimeError as err:   # the server went away during the clean-up
                log.error(f'server lost while removing the sensors ({err})')
                code = 1
        # A measurement that was already being converted may still publish: give it a moment to finish.
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
