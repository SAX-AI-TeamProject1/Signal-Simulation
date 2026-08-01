#!/usr/bin/env bash
# Runs Gazebo alone against the navi_factory world (no rosbridge) — for
# checking world placement, tracks, LiDAR, etc. without the ROS 2 side.
# Used by VS Code task "2. Gazebo 실행 (navi_factory 월드)".
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

source /opt/ros/jazzy/setup.bash

pkill -TERM -f "gz sim server" 2>/dev/null || true
sleep 1

export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$REPO_ROOT/worlds/navi_factory/models"
gz sim --render-engine ogre2 worlds/navi_factory/world/navi_factory/navi_factory.sdf -r
