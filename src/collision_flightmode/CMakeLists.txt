cmake_minimum_required(VERSION 3.8)
project(collision_flightmode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(vdb_mapping_interfaces REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Add executable
add_executable(collision_autonomous_mode
  src/main.cpp
  src/collision_autonomous_mode.cpp
)

# Target dependencies
ament_target_dependencies(collision_autonomous_mode
  rclcpp
  px4_ros2_cpp
  px4_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
  std_srvs
  pcl_conversions
  vdb_mapping_interfaces
)

# Link PCL libraries
target_link_libraries(collision_autonomous_mode ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  collision_autonomous_mode
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