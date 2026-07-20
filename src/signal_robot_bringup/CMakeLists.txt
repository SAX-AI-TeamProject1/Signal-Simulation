cmake_minimum_required(VERSION 3.8)
project(signal_robot_bringup)

find_package(ament_cmake REQUIRED)

# This package ships no code — only files that other things read at run time.
# They have to be installed into share/, because get_package_share_directory()
# (used by the launch file) points at the install tree, not at these sources.
# Build with --symlink-install to avoid rebuilding after every SDF tweak.
install(
  DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
