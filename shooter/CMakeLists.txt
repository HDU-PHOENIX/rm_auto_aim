cmake_minimum_required(VERSION 3.8)
project(shooter)


SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_EXTENSIONS ON)
SET(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-g)
  add_compile_options(-march=native -O3)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen )

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN auto_aim::ShooterNode
    EXECUTABLE shooter_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
    INSTALL_TO_SHARE
)

