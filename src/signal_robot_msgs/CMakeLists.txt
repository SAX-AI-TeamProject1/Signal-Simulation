# An interface-only package. rosidl generates Python and C++ bindings from the
# .msg files below, which is what lets this repository and the perception
# repository agree on a wire format without sharing any code.

cmake_minimum_required(VERSION 3.8)
project(signal_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SignalCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
