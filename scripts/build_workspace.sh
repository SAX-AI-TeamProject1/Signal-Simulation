#!/usr/bin/env bash
# Installs ROS 2 Jazzy (if missing) and builds the colcon workspace.
# Used by the "1. 워크스페이스 빌드 (colcon build)" VS Code task, and can be run
# directly: ./scripts/build_workspace.sh
set -e

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[setup] ROS 2 Jazzy not found — installing (needs sudo, downloads several GB)..."

    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    sudo apt-get update

    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F\" '{print $4}')
    CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
    sudo apt-get install -y /tmp/ros2-apt-source.deb

    sudo apt-get update
    sudo apt-get upgrade -y
    sudo apt-get install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz ros-jazzy-rosbridge-suite ros-jazzy-twist-mux
fi

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
