cmake_minimum_required(VERSION 3.8)
project(camera)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)
# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")

##
## Demos
##



# A single program with one of each of the image pipeline demo nodes.
# add_executable(camera_node
#   src/camera_node.cpp src/mindvision.cpp)
# target_link_libraries(camera_node
#   rclcpp::rclcpp
#   ${builtin_interfaces_TARGETS}
#   ${sensor_msgs_TARGETS}
#   MVSDK
#   opencv_core
#   opencv_highgui
#   opencv_imgproc)
  
# # A single program with one of each of the image pipeline demo nodes.
# add_executable(subscriber_test_node
#   src/subscriber_test_node.cpp)
# target_link_libraries(subscriber_test_node
#   rclcpp::rclcpp
#   ${builtin_interfaces_TARGETS}
#   ${sensor_msgs_TARGETS}
#   opencv_core
#   opencv_highgui
#   opencv_imgproc)

# A stand alone node which produces images from a camera using OpenCV.
add_executable(camera_test
  src/camera_test.cpp src/camera_node.cpp src/subscriber_test_node.cpp src/mindvision.cpp)
target_link_libraries(camera_test
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  MVSDK
  opencv_core
  opencv_highgui
  opencv_imgproc)

install(TARGETS
  camera_test
#   camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
