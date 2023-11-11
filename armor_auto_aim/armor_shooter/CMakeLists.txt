cmake_minimum_required(VERSION 3.8)
project(armor_shooter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ######################
# # Find dependencies ##
# ######################
find_package(ament_cmake_auto REQUIRED)
FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
ament_auto_find_build_dependencies()
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# ##########
# # Build ##
# ##########
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN armor::ArmorShooterNode
  EXECUTABLE armor_shooter_node
)

# ############
# # Install ##
# ############
ament_auto_package(
  INSTALL_TO_SHARE
)
