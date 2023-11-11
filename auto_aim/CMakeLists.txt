cmake_minimum_required(VERSION 3.8)

project(auto_aim)
SET(MAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
