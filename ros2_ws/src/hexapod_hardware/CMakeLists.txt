cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware)

find_package(ament_cmake REQUIRED)

# P6 stub: no hardware_interface::SystemInterface plugin exists yet -- see
# include/hexapod_hardware/hexapod_system.hpp and README.md. Only the
# planned header and config layout are installed so the workspace builds
# and the package is addressable from other packages' package.xml.
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
