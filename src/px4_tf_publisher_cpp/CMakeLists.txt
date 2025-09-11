cmake_minimum_required(VERSION 3.5)
project(px4_tf_publisher_cpp)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})
add_executable(px4_tf_publisher
        src/main.cpp)
ament_target_dependencies(px4_tf_publisher 
    rclcpp
    px4_ros2_cpp
    px4_msgs
    tf2
    tf2_ros
    geometry_msgs
    nav_msgs
    px4_ros_com
    Eigen3)

install(TARGETS
        px4_tf_publisher
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()