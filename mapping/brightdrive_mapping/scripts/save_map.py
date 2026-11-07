#!/usr/bin/env python3
"""Call the backend's save service and exit 0 only when it reports success.

  ros2 run brightdrive_mapping save_map.py [--service /mapping/save_map] [--timeout 10]

The replay launch runs it when the bag ends: the save runs under the backend's lock and the response comes back
when the files are written, so whatever follows (the dynamic filter, the shutdown) starts on finished files.
Exit 1 when the service is not up within --timeout s or answers success=false (no keyframe yet); the backend's
message is printed either way.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--service', default='/mapping/save_map')
    ap.add_argument('--timeout', type=float, default=10.0, help='seconds to wait for the service to exist')
    args, _ = ap.parse_known_args()  # --ros-args and friends go to rclpy
    rclpy.init(args=sys.argv)
    node = Node('save_map_client')
    client = node.create_client(Trigger, args.service)
    if not client.wait_for_service(timeout_sec=args.timeout):
        print(f'{args.service}: no service within {args.timeout:.0f} s', file=sys.stderr)
        rclpy.shutdown()
        return 1
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)  # no timeout: the save takes as long as it takes
    res = future.result()
    print(f'{args.service}: success={res.success}, {res.message}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res.success else 1


if __name__ == '__main__':
    sys.exit(main())
