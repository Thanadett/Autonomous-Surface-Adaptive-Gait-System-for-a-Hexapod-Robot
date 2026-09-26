cmake_minimum_required(VERSION 3.8)
project(hexapod_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContacts.msg"
  "msg/TerrainFeatures.msg"
  "msg/GaitSelection.msg"
  "msg/LocomotionState.msg"
  "srv/SetGaitMode.srv"
  DEPENDENCIES std_msgs
)

ament_package()
