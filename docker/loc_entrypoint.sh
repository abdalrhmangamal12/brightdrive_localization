#!/bin/bash
# Humble + the repo's install/ if built, then the command.
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/src/brightdrive_localization/install/setup.bash ]; then
  source /ws/src/brightdrive_localization/install/setup.bash
fi
exec "$@"
