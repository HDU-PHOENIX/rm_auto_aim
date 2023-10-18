cmake_minimum_required(VERSION 3.8)
project(auto_aim)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use clang as compiler
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")

# Set compiler flags
add_compile_options(-Wall -Wextra -Wpedantic)

add_compile_options(-g)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies()


ament_auto_add_executable(${PROJECT_NAME}
    DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MVSDK)

ament_auto_package()
