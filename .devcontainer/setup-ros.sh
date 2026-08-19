#!/usr/bin/env bash
# Source ROS 2 and the workspace overlay in every interactive shell.
#
# VS Code devcontainers bypass the base image's /ros_entrypoint.sh, so without
# this a new terminal has no `ros2` on PATH at all.
set -e

WS=/workspaces/RoboticsLab/ros_ws

grep -qxF "source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash" ~/.bashrc \
  || echo "source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash" >> ~/.bashrc

# The overlay only exists after the first build; guard it so a fresh clone
# does not print an error on every new shell.
grep -qxF "[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash" ~/.bashrc \
  || echo "[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash" >> ~/.bashrc

echo "ROS 2 environment ready."
