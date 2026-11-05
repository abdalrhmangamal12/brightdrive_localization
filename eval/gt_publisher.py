#!/usr/bin/env python3
"""The TRUE pose and velocity of our car, for evaluation only.

Publishes nav_msgs/Odometry on /eval/ground_truth at every world tick (100 Hz at tick_s 0.01): the pose of
base_link in the map frame (CARLA's world with y flipped), the twist in base_link. The estimator never subscribes
to it (AGENTS.md: ground truth only in eval/); the evaluation compares the estimator's output with it. One message
per tick from the same callback that stamps /clock in the bridge, so the two counts are equal in a bag.

Inside the sim container (ROS 2 sourced; run_inside.sh starts it):
  python3 -u eval/gt_publisher.py
"""
# Maths, stopping on Ctrl-C, wait loops, paths.
import math
import signal
import time
from pathlib import Path

# CARLA API, ROS 2, the shared car config.
import carla
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

# Repo root (this file is eval/gt_publisher.py); the car's description lives in sim/config/.
ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'sim'


def ros_time(seconds):
    """Simulation seconds -> ROS Time: whole nanoseconds first, then the split (carries a rounded-up second)."""
    sec, nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def rotation_matrix(roll, pitch, yaw):
    """Roll, pitch, yaw (radians, ROS) -> 3x3 rotation, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def odometry(actor_snapshot, stamp):
    """One CARLA actor snapshot -> Odometry in the ROS map frame."""
    t = actor_snapshot.get_transform()
    msg = Odometry()
    msg.header.stamp = ros_time(stamp)
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_link'
    # Position: CARLA's y points right, ROS's left.
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = \
        t.location.x, -t.location.y, t.location.z
    # Orientation: degrees -> radians, pitch and yaw flip with the y axis.
    roll, pitch, yaw = math.radians(t.rotation.roll), -math.radians(t.rotation.pitch), -math.radians(t.rotation.yaw)
    # Quaternion from roll, pitch, yaw (half angles).
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
    msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
    msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
    msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
    # Velocities from CARLA are in the world frame: linear m/s, angular deg/s. Flip y for the linear one; for a
    # rotation, flipping the y axis flips the x and z components instead.
    v = actor_snapshot.get_velocity()
    w = actor_snapshot.get_angular_velocity()
    v_map = np.array([v.x, -v.y, v.z])
    w_map = np.radians([-w.x, w.y, -w.z])
    # Odometry twist is expressed in the car's frame (child_frame_id): rotate from map into base_link.
    R = rotation_matrix(roll, pitch, yaw)
    (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z) = R.T @ v_map
    (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z) = R.T @ w_map
    return msg


def main():
    # Our car's role_name comes from the shared config.
    role_name = yaml.safe_load(open(SIM / 'config' / 'sensors.yaml'))['hero']['role_name']
    # Ctrl-C is left to Python (KeyboardInterrupt below), so the CARLA callback is removed before ROS stops.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # run_inside.sh starts us in the background, and bash starts background jobs with Ctrl-C ignored: switch it
    # back on, and treat a stop request (SIGTERM, e.g. docker stop) the same way.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = rclpy.create_node('ground_truth')
    pub = node.create_publisher(Odometry, '/eval/ground_truth', 10)
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world, cb = None, None
    try:
        while rclpy.ok():
            node.get_logger().info(f'waiting for a vehicle with role_name={role_name} ...')
            hero = None
            while hero is None:
                world = client.get_world()
                hero = next((a for a in world.get_actors().filter('vehicle.*')
                             if a.attributes.get('role_name') == role_name), None)
                time.sleep(0.5)
            node.get_logger().info(f'publishing /eval/ground_truth for {hero.type_id} (id {hero.id})')
            hero_id = hero.id

            # Every tick: find our car in the world snapshot (same instant as the sensors) and publish it.
            def on_tick(snapshot):
                actor = snapshot.find(hero_id)
                if actor is not None:
                    pub.publish(odometry(actor, snapshot.timestamp.elapsed_seconds))
            cb = world.on_tick(on_tick)
            # Until the scenario destroys the car (the client's copy of the actor list, refreshed by every tick).
            while rclpy.ok() and hero_id in [a.id for a in world.get_actors().filter('vehicle.*')]:
                time.sleep(0.5)
            world.remove_on_tick(cb)
            cb = None
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the CARLA callback first, then ROS.
        if cb is not None:
            world.remove_on_tick(cb)
        # A tick that was already being handled may still publish: give it a moment to finish.
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
