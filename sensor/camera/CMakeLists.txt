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

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MVSDK)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN sensor::CameraNode
    EXECUTABLE camera_node
)

#############
## Testing ##
######### ####

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  ament_add_gtest(${PROJECT_NAME}-test test/camera_test.cpp)
  target_include_directories(${PROJECT_NAME}-test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}-test ${OpenCV_LIBS} ${PROJECT_NAME})

endif()

ament_auto_package(
    INSTALL_TO_SHARE
)
