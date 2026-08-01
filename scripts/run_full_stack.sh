#!/usr/bin/env bash
# Runs rosbridge_server + Gazebo (navi_factory world) together, so
# Signal-Vision can send hand-signal commands over the rosbridge websocket.
# Stopping this script (or the task) also tears down rosbridge.
# Used by VS Code task "3. 전체 프로세스 실행 (rosbridge + Gazebo)".

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

source /opt/ros/jazzy/setup.bash

pkill -TERM -f "gz sim server" 2>/dev/null || true
sleep 1

# Kill the whole process group (rosbridge included) when this script exits.
trap 'kill -TERM -$$ 2>/dev/null' EXIT

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 2

export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$REPO_ROOT/worlds/navi_factory/models"
gz sim --render-engine ogre2 worlds/navi_factory/world/navi_factory/navi_factory.sdf -r
