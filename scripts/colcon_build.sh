#!/usr/bin/env bash
# Plain rebuild — no ROS 2 install check (that's build_workspace.sh's job).
# Used by the "2. colcon build (ROS 설치 확인 없이 재빌드)" VS Code task, and can be run
# directly: ./scripts/colcon_build.sh
set -e

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
