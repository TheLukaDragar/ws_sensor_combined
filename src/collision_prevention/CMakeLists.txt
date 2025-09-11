cmake_minimum_required(VERSION 3.8)
project(collision_prevention)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(collision_prevention_node
  src/main.cpp
  src/collision_prevention_node.cpp
)

add_executable(obstacle_to_laser_node
  src/obstacle_to_laser_main.cpp
  src/obstacle_to_laser_node.cpp
)

# Target dependencies
ament_target_dependencies(collision_prevention_node
  rclcpp
  sensor_msgs
  px4_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(obstacle_to_laser_node
  rclcpp
  sensor_msgs
  px4_msgs
  tf2
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  collision_prevention_node
  obstacle_to_laser_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()