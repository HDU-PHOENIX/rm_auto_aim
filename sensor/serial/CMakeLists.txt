cmake_minimum_required(VERSION 3.8)
project(serial)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use clang as compiler
# set(CMAKE_C_COMPILER clang)
# set(CMAKE_CXX_COMPILER clang++)
# set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")

# Set compiler flags
add_compile_options(-Wall -Wextra -Wpedantic)

add_compile_options(-g)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN sensor::SerialNode
    EXECUTABLE serial_node
)

ament_auto_package(
    INSTALL_TO_SHARE
    launch
)
