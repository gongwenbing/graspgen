#!/usr/bin/env bash
# Launch a fresh, fully automatic D455 cube-to-basket simulation.
set -e
task_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$task_workspace/install/setup.bash"
source "$task_workspace/piper_ros/install_d455/local_setup.bash"
export ROS_DOMAIN_ID="${PIPER_SIM_DOMAIN_ID:-188}"
exec ros2 launch piper_gazebo piper_auto_cube.launch.py "$@"
