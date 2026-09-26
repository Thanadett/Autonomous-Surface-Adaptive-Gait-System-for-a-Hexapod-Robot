cmake_minimum_required(VERSION 3.8)
project(hexapod_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/points_frame_relabel.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
