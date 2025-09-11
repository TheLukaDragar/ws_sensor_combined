cmake_minimum_required(VERSION 3.8)
project(laserscan_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############
## Packages ##
##############

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
  Eigen3
)



###########
## Build ##
###########

#===========
# includes =
#===========

include_directories(include)

#============
# libraries =
#============

add_library(polar_to_cartesian_matrix_cache
    src/polar_to_cartesian_matrix_cache.cpp
)

add_library(tf_rosmsg_eigen_conversions
    src/tf_rosmsg_eigen_conversions.cpp
)

add_library(tf_collector
    src/tf_collector.cpp
)

add_library(laserscan_to_pointcloud
    src/laserscan_to_pointcloud.cpp
)

#==============
# executables =
#==============

add_executable(laserscan_to_pointcloud_assembler
    src/laserscan_to_ros_pointcloud.cpp
    src/laserscan_to_pointcloud_assembler.cpp
    src/laserscan_to_pointcloud_assembler_node.cpp
)

#=================
# libraries link =
#=================

ament_target_dependencies(polar_to_cartesian_matrix_cache ${DEPENDENCIES})

ament_target_dependencies(tf_rosmsg_eigen_conversions ${DEPENDENCIES})

target_link_libraries(tf_collector tf_rosmsg_eigen_conversions)
ament_target_dependencies(tf_collector ${DEPENDENCIES})

target_link_libraries(laserscan_to_pointcloud tf_collector polar_to_cartesian_matrix_cache)
ament_target_dependencies(laserscan_to_pointcloud ${DEPENDENCIES})

#===================
# executables link =
#===================

target_link_libraries(laserscan_to_pointcloud_assembler laserscan_to_pointcloud)
ament_target_dependencies(laserscan_to_pointcloud_assembler ${DEPENDENCIES})

#############
## Install ##
#############

install(TARGETS
    polar_to_cartesian_matrix_cache
    tf_rosmsg_eigen_conversions
    tf_collector
    laserscan_to_pointcloud
    laserscan_to_pointcloud_assembler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_export_include_directories(include)
ament_export_libraries(laserscan_to_pointcloud tf_collector polar_to_cartesian_matrix_cache tf_rosmsg_eigen_conversions)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
