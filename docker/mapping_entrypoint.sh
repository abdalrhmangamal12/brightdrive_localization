#!/bin/bash
# Humble + FAST-LIO underlay + the repo's install/ if built, then the command.
set -e
source /opt/ros/humble/setup.bash
source /opt/fast_lio_ws/install/setup.bash
if [ -f /ws/src/brightdrive_localization/install/setup.bash ]; then
  source /ws/src/brightdrive_localization/install/setup.bash
fi
exec "$@"
