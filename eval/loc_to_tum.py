#!/usr/bin/env python3
"""Export the localizer's pose to a TUM trajectory in the IMU frame, the evo estimate for the localization runs.

/localization/pose is base_link in map; ground truth goes through eval/gt_to_tum.py the same way, so both files
hold T_map_imu = T_map_base * T_base_imu (the identity in the current setup). Same reader and writer as
gt_to_tum.py: PoseWithCovarianceStamped and Odometry both carry pose.pose.

Inside the loc container (ROS sourced):
  python3 eval/loc_to_tum.py <run_bag> <out.txt> [--topic /localization/pose] [--sensors sim/config/sensors.yaml]
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gt_to_tum  # noqa: E402

if __name__ == '__main__':
    if '--topic' not in sys.argv:
        sys.argv += ['--topic', '/localization/pose']
    gt_to_tum.main()
