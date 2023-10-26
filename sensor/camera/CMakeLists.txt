cmake_minimum_required(VERSION 3.8)
project(camera)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set compiler flags
add_compile_options(-Wall -Wextra -Wpedantic)

add_compile_options(-g)

find_package(ament_cmake_auto REQUIRED) 
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MVSDK)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN sensor::CameraNode
    EXECUTABLE camera_node
)

ament_auto_package(
    INSTALL_TO_SHARE
)
